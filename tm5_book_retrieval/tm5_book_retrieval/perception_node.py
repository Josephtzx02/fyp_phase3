import math
import os
import joblib
import numpy as np
import pandas as pd
import pyrealsense2 as rs
import cv2
import rclpy
from rclpy.node import Node
from tm5_book_msgs.msg import GraspTarget as GraspTargetMsg
from tm5_book_msgs.msg import Mission as MissionMsg

from .shelf_location import ShelfLocation

from ultralytics import YOLO
from .coordinate_transform import pixel_to_camera
from .grasp_target import create_grasp_target


PROJECT_ROOT = os.path.expanduser("~/fyp_phase3")
MODEL_DIR = os.path.join(PROJECT_ROOT, "src", "tm5_book_retrieval", "models")

MODEL_PATH = os.path.join(MODEL_DIR, "best.pt")
WIDTH_MODEL_PATH = os.path.join(MODEL_DIR, "width_model.pkl")
WIDTH_FEATURES_PATH = os.path.join(MODEL_DIR, "width_features.pkl")
WEIGHT_MODEL_PATH = os.path.join(MODEL_DIR, "weight_model_D_huber.pkl")
WEIGHT_FEATURES_PATH = os.path.join(MODEL_DIR, "weight_features_D.pkl")

CONF_THRESHOLD = 0.8
MIN_VALID_DEPTH_MM = 370
MAX_VALID_DEPTH_MM = 405
CENTER_ZONE_RATIO = 0.15

CLASS_NAMES = {
    0: "book",
    1: "shelf"
}

BOOK_SIZE_PRIORS = [
    ("US_Pocket", 175, 108),
    ("Mass_Market", 178, 110),
    ("B6", 176, 125),
    ("B6_Slim", 176, 110),
    ("US_Trade_Small", 203, 127),
    ("A5", 210, 148),
    ("Digest", 216, 140),
    ("Demy_UK", 216, 138),
    ("US_Standard", 229, 152),
    ("Royal_UK", 234, 156),
    ("B5", 250, 176),
    ("Executive", 267, 184),
    ("Large_Textbook", 280, 215),
    ("A4", 297, 210),
]


def infer_book_size_from_height(h_mm):
    return min(
        BOOK_SIZE_PRIORS,
        key=lambda x: abs(h_mm - x[1])
    )


def make_feature_row(det):
    height_mm = det["height_mm"]
    thickness_mm = det["thickness_mm"]
    depth_mm = det["depth_mm"]
    height_px = det["height_px"]
    thickness_px = det["thickness_px"]

    family, standard_width_prior, height_to_standard_error = \
        infer_book_size_from_height(height_mm)

    row = {
        "height_px": height_px,
        "thickness_px": thickness_px,
        "depth_rs_mm": depth_mm,
        "height_mm_est": height_mm,
        "thickness_mm_est": thickness_mm,
        "aspect_ratio_px": height_px / thickness_px,
        "scale_h": height_mm / height_px,
        "thickness_ratio": thickness_mm / height_mm,
        "pixel_area_proxy": height_px * thickness_px,
        "standard_width_prior": standard_width_prior,
        "height_to_standard_error": height_to_standard_error,
        "prior_width_ratio": standard_width_prior / height_mm,
        "auto_paper_family": family,
        "height_x_thickness_est": height_mm * thickness_mm,
        "height_x_prior_width": height_mm * standard_width_prior,
        "thickness_x_prior_width": thickness_mm * standard_width_prior,
        "depth_x_height": depth_mm * height_mm,
        "thinness_score": height_mm / thickness_mm,
        "px_density_proxy": (height_px * thickness_px) / depth_mm,
        "thickness_to_height_est": thickness_mm / height_mm,
        "thickness_x_depth": thickness_mm * depth_mm,
        "height_x_depth": height_mm * depth_mm,
    }

    for fam, h_std, w_std in BOOK_SIZE_PRIORS:
        row[f"dist_h_{fam}"] = abs(height_mm - h_std)

    return row


def predict_width_weight(det, width_model, width_features,
                          weight_model, weight_features):
    row = make_feature_row(det)

    width_data = {
        col: row[col]
        for col in width_features
    }

    width_df = pd.DataFrame([width_data])

    width_pred = float(
        width_model.predict(width_df)[0]
    )

    row["width_pred_mm"] = width_pred
    row["est_volume_proxy"] = (
        row["height_mm_est"] *
        row["thickness_mm_est"] *
        width_pred
    )

    weight_data = {
        col: row[col]
        for col in weight_features
    }

    weight_df = pd.DataFrame([weight_data])

    weight_pred = float(
        weight_model.predict(weight_df)[0]
    )

    return width_pred, weight_pred

def get_median_depth_mm(depth_frame, cx, cy, patch=7):
    half = patch // 2
    depths = []

    w = depth_frame.get_width()
    h = depth_frame.get_height()

    for dx in range(-half, half + 1):
        for dy in range(-half, half + 1):
            x = int(cx + dx)
            y = int(cy + dy)

            if x < 0 or y < 0 or x >= w or y >= h:
                continue

            try:
                d = depth_frame.get_distance(x, y)
                if d > 0:
                    depths.append(d * 1000)
            except Exception:
                pass

    if not depths:
        return None

    return float(np.median(depths))


def check_center(cx, image_width):
    center_x = image_width / 2
    half_zone = image_width * CENTER_ZONE_RATIO / 2
    return abs(cx - center_x) <= half_zone


def main():
    rclpy.init()
    ros_node = Node("tm5_perception_node")
    shelf_location = [ShelfLocation.BOTTOM]
    def mission_callback(msg):
        try:
            shelf_location[0] = ShelfLocation(msg.source)

            print(
                f"Mission received: "
                f"{msg.source} → {msg.destination}"
            )

            print(
                f"Perception shelf selected: "
                f"{shelf_location[0].value}"
            )

        except ValueError:
            print(
                f"Invalid shelf location received: "
                f"{msg.source}"
            )
            
    mission_sub = ros_node.create_subscription(
        MissionMsg,
        "/tm5/mission",
        mission_callback,
        10
    )
    
    confidence_threshold = CONF_THRESHOLD
    
    grasp_pub = ros_node.create_publisher(
        GraspTargetMsg,
        "/tm5/grasp_target",
        10
    )

    print("Loading FYP models...")

    model = YOLO(MODEL_PATH)

    width_model = joblib.load(WIDTH_MODEL_PATH)
    width_features = joblib.load(WIDTH_FEATURES_PATH)

    weight_model = joblib.load(WEIGHT_MODEL_PATH)
    weight_features = joblib.load(WEIGHT_FEATURES_PATH)

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        1280,
        720,
        rs.format.bgr8,
        30
    )

    config.enable_stream(
        rs.stream.depth,
        848,
        480,
        rs.format.z16,
        30
    )

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    color_stream = profile.get_stream(rs.stream.color)
    intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

    print("RealSense D435i initialized")
    print("Waiting for frames...")

    selected_index = None
    selected_info = None
    selection_valid = False
    detections = []
    click_request = [None]

    def mouse_callback(event, mouse_x, mouse_y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_request[0] = (mouse_x, mouse_y)

    cv2.namedWindow("TM5 Book Selection")
    cv2.setMouseCallback("TM5 Book Selection", mouse_callback)

    try:
        while True:
            rclpy.spin_once(ros_node, timeout_sec=0.0)
        
            frames = align.process(
                pipeline.wait_for_frames()
            )

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            image = np.asanyarray(color_frame.get_data())

            results = model(
                image,
                conf=confidence_threshold,
                verbose=False
            )[0]

            detections = []

            if results.obb:
                for box, conf, cls in zip(
                    results.obb.xywhr,
                    results.obb.conf,
                    results.obb.cls
                ):
                    x, y, bt, bh, angle = box.tolist()

                    angle_deg = math.degrees(angle)

                    if bt > bh:
                        bt, bh = bh, bt
                        angle_deg += 90

                    while angle_deg >= 90:
                        angle_deg -= 180

                    while angle_deg < -90:
                        angle_deg += 180

                    class_name = CLASS_NAMES.get(
                        int(cls),
                        str(int(cls))
                    )

                    if class_name != "book":
                        continue

                    depth_mm = get_median_depth_mm(
                        depth_frame,
                        x,
                        y
                    )

                    center_ok = check_center(
                        x,
                        image.shape[1]
                    )

                    px_t = int(bt)
                    px_h = int(bh)

                    det = {

                        "thickness_px": px_t,
                        "height_px": px_h,
                        "angle": angle_deg,
                        "conf": float(conf),
                        "center_u": float(x),
                        "center_v": float(y),
                        "class_name": class_name,
                        "depth_mm": depth_mm,
                        "center_ok": center_ok,
                        "mode_used": "DEPTH-AWARE",
                        "obb_corners": cv2.boxPoints(
                            (
                                (float(x), float(y)),
                                (float(bt), float(bh)),
                                float(angle_deg)
                            )
                        ).astype(int)
                    }
                    detections.append(det)
            
            # Keep book numbering stable from left to right
            detections.sort(
                key=lambda d: (
                    d["center_u"],
                    d["center_v"]
                )
            )                 

            # Draw detected books
            display = image.copy()

            # Draw the same center working zone used by check_center()
            image_width = display.shape[1]
            center_x = image_width // 2
            half_zone = int(
                image_width * CENTER_ZONE_RATIO / 2
            )
            zone_left = center_x - half_zone
            zone_right = center_x + half_zone

            for x_zone in (zone_left, zone_right):
                for y_zone in range(0, display.shape[0], 24):
                    cv2.line(
                        display,
                        (x_zone, y_zone),
                        (
                            x_zone,
                            min(y_zone + 10, display.shape[0] - 1)
                        ),
                        (255, 255, 0),
                        1
                    )

            cv2.putText(
                display,
                "CENTER ZONE",
                (zone_left + 8, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1
            )

            for i, det in enumerate(detections):
                cx = int(det["center_u"])
                cy = int(det["center_v"])
                corners = det["obb_corners"]

                box_color = (
                    (0, 255, 0)
                    if det["center_ok"]
                    else (0, 255, 255)
                )

                cv2.polylines(
                    display,
                    [corners],
                    True,
                    box_color,
                    2
                )

                cv2.circle(
                    display,
                    (cx, cy),
                    5,
                    box_color,
                    -1
                )

                cv2.putText(
                    display,
                    f"BOOK {i + 1}",
                    (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    box_color,
                    2
                )

            # Highlight selected book
            if selected_info is not None:
                corners = selected_info["obb_corners"]
                cx = int(selected_info["center_u"])
                cy = int(selected_info["center_v"])

                cv2.polylines(
                    display,
                    [corners],
                    True,
                    (0, 0, 255),
                    3
                )

                cv2.putText(
                    display,
                    "SELECTED",
                    (cx + 12, cy + 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2
                )

            # Create information panel
            panel_w = 360
            panel = np.zeros(
                (display.shape[0], panel_w, 3),
                dtype=np.uint8
            )

            cv2.putText(
                panel,
                "TM5 BOOK RETRIEVAL",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (255, 255, 255),
                2
            )

            cv2.line(
                panel,
                (20, 55),
                (panel_w - 20, 55),
                (100, 100, 100),
                1
            )

            cv2.putText(
                panel,
                f"BOOKS DETECTED: {len(detections)}",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (220, 220, 220),
                1
            )

            cv2.putText(
                panel,
                f"CONFIDENCE THRESHOLD: {confidence_threshold:.2f}",
                (20, 112),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (170, 170, 170),
                1
            )

            y = 145

            if selected_index is None:
                cv2.putText(
                    panel,
                    "NO BOOK SELECTED",
                    (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (180, 180, 180),
                    2
                )

                cv2.putText(
                    panel,
                    "Click a detected book",
                    (20, y + 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (180, 180, 180),
                    1
                )

            elif selected_info is not None:
                det = selected_info

                cv2.putText(
                    panel,
                    f"SELECTED: BOOK {selected_index + 1}",
                    (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

                y += 42

                sections = [
                    (
                        "DETECTION",
                        [
                            f"Confidence    {det['conf']:.3f}",
                            f"Pixel Center  ({det['center_u']:.1f}, {det['center_v']:.1f})"
                        ]
                    ),
                    (
                        "GEOMETRY",
                        [
                            f"Height        {det['height_mm']:.1f} mm"
                            if "height_mm" in det else "Height        N/A",
                            f"Width         {det['pred_width_mm']:.1f} mm"
                            if "pred_width_mm" in det else "Width         N/A",
                            f"Thickness     {det['thickness_mm']:.1f} mm"
                            if "thickness_mm" in det else "Thickness     N/A",
                            f"Angle         {det['angle']:.1f} deg"
                        ]
                    ),
                    (
                        "DEPTH",
                        [
                            f"Distance      {det['depth_mm']:.1f} mm"
                            if det["depth_mm"] is not None
                            else "Distance      N/A"
                        ]
                    ),
                    (
                        "PREDICTION",
                        [
                            f"Weight        {det['pred_weight_g']:.1f} g"
                            if "pred_weight_g" in det else "Weight        N/A"
                        ]
                    ),
                    (
                        "STATUS",
                        [
                            f"Center Zone   {'OK' if det['center_ok'] else 'OUTSIDE'}",
                            f"Selection     {'VALID' if selection_valid else 'NOT VALID'}"
                        ]
                    )
                ]

                for title, values in sections:
                    cv2.putText(
                        panel,
                        title,
                        (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (160, 160, 160),
                        1
                    )

                    y += 24

                    for text in values:
                        cv2.putText(
                            panel,
                            text,
                            (25, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.52,
                            (255, 255, 255),
                            1
                        )
                        y += 27

                    y += 8

            cv2.line(
                panel,
                (20, display.shape[0] - 95),
                (panel_w - 20, display.shape[0] - 95),
                (100, 100, 100),
                1
            )

            cv2.putText(
                panel,
                "CLICK  Select book",
                (20, display.shape[0] - 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (220, 220, 220),
                1
            )

            cv2.putText(
                panel,
                "C  Clear selection    Q  Quit",
                (20, display.shape[0] - 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (220, 220, 220),
                1
            )

            # Combine camera and information panel
            display = np.hstack((display, panel))

            cv2.imshow(
                "TM5 Book Selection",
                display
            )
            
            # Handle mouse selection
            if click_request[0] is not None:
                mx, my = click_request[0]
                click_request[0] = None

                if detections:
                    clicked_indices = []

                    for i, det in enumerate(detections):
                        corners = np.array(
                            det["obb_corners"],
                            dtype=np.float32
                        )

                        inside = cv2.pointPolygonTest(
                            corners,
                            (float(mx), float(my)),
                            False
                        )

                        if inside >= 0:
                            clicked_indices.append(i)

                    if not clicked_indices:
                        print(
                            f"Click ({mx}, {my}) is outside all OBBs. "
                            "Current selection unchanged."
                        )

                        continue

                    selected_index = min(
                        clicked_indices,
                        key=lambda i:
                        (
                            detections[i]["center_u"] - mx
                        ) ** 2 +
                        (
                            detections[i]["center_v"] - my
                        ) ** 2
                    )

                    det = detections[selected_index]
                    selected_info = det.copy()

                    depth_mm = det["depth_mm"]
                    center_ok = det["center_ok"]

                    depth_ok = (
                        depth_mm is not None and
                        MIN_VALID_DEPTH_MM <= depth_mm <= MAX_VALID_DEPTH_MM
                    )

                    selection_valid = depth_ok and center_ok

                    print("\n========== BOOK SELECTED ==========")
                    print(f"Index      : {selected_index + 1}")
                    print(f"Confidence : {det['conf']:.3f}")
                    print(f"Depth      : {depth_mm} mm")
                    print(f"Center OK  : {center_ok}")
                    print("===================================")

                    if not depth_ok or not center_ok:
                        print(
                            "SELECTED BOOK INVALID: "
                            f"depth={depth_mm} mm, "
                            f"center={center_ok}"
                        )
                        continue

                    px_t = det["thickness_px"]
                    px_h = det["height_px"]

                    thickness_mm = (
                        px_t * depth_mm / intrinsics.fx
                    )

                    height_mm = (
                        px_h * depth_mm / intrinsics.fy
                    )

                    camera_coord = pixel_to_camera(
                        u=det["center_u"],
                        v=det["center_v"],
                        depth_mm=depth_mm,
                        intrinsics=intrinsics
                    )

                    det["thickness_mm"] = thickness_mm
                    det["height_mm"] = height_mm
                    det["camera_coord"] = camera_coord
                    
                    selected_info.update(det)

                    print(f"Camera XYZ : {camera_coord}")

                    pred_width, pred_weight = predict_width_weight(
                        det,
                        width_model,
                        width_features,
                        weight_model,
                        weight_features
                    )

                    det["pred_width_mm"] = pred_width
                    det["pred_weight_g"] = pred_weight
                    
                    selected_info.update(det)

                    grasp_target = create_grasp_target(det, shelf_location[0])

                    if grasp_target is not None:
                        msg = GraspTargetMsg()

                        msg.book_position.x = grasp_target.book_x_mm / 1000.0
                        msg.book_position.y = grasp_target.book_y_mm / 1000.0
                        msg.book_position.z = grasp_target.book_z_mm / 1000.0

                        msg.grasp_position.x = grasp_target.grasp_x_mm / 1000.0
                        msg.grasp_position.y = grasp_target.grasp_y_mm / 1000.0
                        msg.grasp_position.z = grasp_target.grasp_z_mm / 1000.0

                        msg.approach_position.x = grasp_target.approach_x_mm / 1000.0
                        msg.approach_position.y = grasp_target.approach_y_mm / 1000.0
                        msg.approach_position.z = grasp_target.approach_z_mm / 1000.0

                        msg.width_mm = grasp_target.width_mm
                        msg.height_mm = grasp_target.height_mm
                        msg.thickness_mm = grasp_target.thickness_mm
                        msg.angle_deg = grasp_target.angle_deg

                        msg.camera_position.x = grasp_target.camera_x_mm / 1000.0
                        msg.camera_position.y = grasp_target.camera_y_mm / 1000.0
                        msg.camera_position.z = grasp_target.camera_z_mm / 1000.0

                        grasp_pub.publish(msg)

                        print(
                            f"Published GraspTarget: "
                            f"({msg.grasp_position.x:.3f}, "
                            f"{msg.grasp_position.y:.3f}, "
                            f"{msg.grasp_position.z:.3f}) m"
                        )

                        print(f"Width      : {pred_width:.1f} mm")
                        print(f"Weight     : {pred_weight:.1f} g")
                        print(f"Angle      : {det['angle']:.2f} deg")
                        print("===================================\n")

            key = cv2.waitKey(1) & 0xFF

            if key == ord("w"):
                confidence_threshold = min(
                    0.95,
                    confidence_threshold + 0.05
                )
                print(
                    f"Confidence threshold: "
                    f"{confidence_threshold:.2f}"
                )

            elif key == ord("x"):
                confidence_threshold = max(
                    0.10,
                    confidence_threshold - 0.05
                )
                print(
                    f"Confidence threshold: "
                    f"{confidence_threshold:.2f}"
                )

            if key == ord("q"):
                break

            if key == ord("c"):
                selected_index = None
                selected_info = None
                selection_valid = False
                click_request[0] = None
                print("Selection cleared.")

    except KeyboardInterrupt:
        print("\nStopping perception node.")

    finally:
        pipeline.stop()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
