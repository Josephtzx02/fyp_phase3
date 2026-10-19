import tkinter as tk

import rclpy
from rclpy.node import Node

from tm5_book_msgs.msg import Mission
from std_msgs.msg import String


class RobotControlNode(Node):
    def __init__(self):
        super().__init__("robot_control_ui")

        self.mission_publisher = self.create_publisher(
            Mission,
            "/tm5/mission",
            10
        )

        self.home_publisher = self.create_publisher(
            String,
            "/tm5/home",
            10
        )

        self.status_subscription = self.create_subscription(
            String,
            "/tm5/status",
            self.status_callback,
            10
        )

        self.ui = None

    def set_ui(self, ui):
        self.ui = ui

    def status_callback(self, msg):
        if self.ui is not None:
            self.ui.update_status(msg.data)

    def publish_mission(self, source, destination):
        msg = Mission()
        msg.source = source
        msg.destination = destination

        self.mission_publisher.publish(msg)

        self.get_logger().info(
            f"Mission published: {source} -> {destination}"
        )

    def publish_home(self, location):
        msg = String()
        msg.data = location

        self.home_publisher.publish(msg)

        self.get_logger().info(
            f"Home command published: {location}"
        )


class RobotControlUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node

        self.root.title("TM5 Robot Control")
        self.root.geometry("500x500")
        self.root.resizable(False, False)

        # Current mission selections
        self.source = None
        self.destination = None
        self.home = None
        self.mission_locked = False
        self.operation_type = None

        self.selected_button_bg = "#4A90E2"
        self.selected_button_fg = "white"

        title = tk.Label(
            root,
            text="TM5 ROBOT CONTROL",
            font=("Arial", 24, "bold")
        )
        title.pack(pady=(25, 30))

        # ============================================================
        # SOURCE
        # ============================================================

        source_label = tk.Label(
            root,
            text="SOURCE",
            font=("Arial", 16, "bold")
        )
        source_label.pack()

        source_frame = tk.Frame(root)
        source_frame.pack(pady=(10, 25))

        self.source_top_button = tk.Button(
            source_frame,
            text="TOP",
            width=12,
            height=2,
            font=("Arial", 14),
            command=lambda: self.set_source("top")
        )
        self.source_top_button.pack(side=tk.LEFT, padx=10)

        self.source_bottom_button = tk.Button(
            source_frame,
            text="BOTTOM",
            width=12,
            height=2,
            font=("Arial", 14),
            command=lambda: self.set_source("bottom")
        )
        self.source_bottom_button.pack(side=tk.LEFT, padx=10)

        # ============================================================
        # DESTINATION
        # ============================================================

        destination_label = tk.Label(
            root,
            text="DESTINATION",
            font=("Arial", 16, "bold")
        )
        destination_label.pack()

        destination_frame = tk.Frame(root)
        destination_frame.pack(pady=(10, 25))

        self.destination_top_button = tk.Button(
            destination_frame,
            text="TOP",
            width=12,
            height=2,
            font=("Arial", 14),
            command=lambda: self.set_destination("top")
        )
        self.destination_top_button.pack(side=tk.LEFT, padx=10)

        self.destination_bottom_button = tk.Button(
            destination_frame,
            text="BOTTOM",
            width=12,
            height=2,
            font=("Arial", 14),
            command=lambda: self.set_destination("bottom")
        )
        self.destination_bottom_button.pack(side=tk.LEFT, padx=10)

        # ============================================================
        # INITIAL BUTTON STATES
        # ============================================================

        self.default_button_bg = self.source_top_button.cget("bg")
        self.default_button_fg = self.source_top_button.cget("fg")

        self.update_button_states()

        # ============================================================
        # ROBOT HOME
        # ============================================================

        home_label = tk.Label(
            root,
            text="ROBOT HOME",
            font=("Arial", 16, "bold")
        )
        home_label.pack()

        home_frame = tk.Frame(root)
        home_frame.pack(pady=(10, 25))

        self.home_top_button = tk.Button(
            home_frame,
            text="TOP",
            width=12,
            height=2,
            font=("Arial", 14),
            command=lambda: self.set_home("top")
        )
        self.home_top_button.pack(side=tk.LEFT, padx=10)

        self.home_bottom_button = tk.Button(
            home_frame,
            text="BOTTOM",
            width=12,
            height=2,
            font=("Arial", 14),
            command=lambda: self.set_home("bottom")
        )
        self.home_bottom_button.pack(side=tk.LEFT, padx=10)

        self.update_home_button_states()

        # ============================================================
        # STATUS
        # ============================================================

        status_frame = tk.Frame(root)
        status_frame.pack(pady=(5, 10))

        self.status_prefix_label = tk.Label(
            status_frame,
            text="Status:",
            font=("Arial", 13),
            fg="black"
        )
        self.status_prefix_label.pack(side=tk.LEFT)

        self.status_label = tk.Label(
            status_frame,
            text="READY",
            font=("Arial", 13),
            fg="green"
        )
        self.status_label.pack(side=tk.LEFT, padx=(5, 0))

    # ================================================================
    # SOURCE / DESTINATION / HOME
    # ================================================================

    def set_home(self, location):
        self.home = location
        self.operation_type = "home"

        self.ros_node.publish_home(location)

        self.status_label.config(
            text=f"HOME = {location.upper()}"
        )

        self.update_home_button_states()

    def update_status(self, status):
        if status == "BUSY":
            self.status_label.config(
                text="BUSY",
                fg="orange"
            )

            self.source_top_button.config(state=tk.DISABLED)
            self.source_bottom_button.config(state=tk.DISABLED)
            self.destination_top_button.config(state=tk.DISABLED)
            self.destination_bottom_button.config(state=tk.DISABLED)
            self.home_top_button.config(state=tk.DISABLED)
            self.home_bottom_button.config(state=tk.DISABLED)

        elif status == "READY":
            self.status_label.config(
                text="READY",
                fg="green"
            )

            if self.operation_type == "mission":
                self.source = None
                self.destination = None
                self.mission_locked = False

            self.update_button_states()

            self.home_top_button.config(state=tk.NORMAL)
            self.home_bottom_button.config(state=tk.NORMAL)

            self.operation_type = None

            self.update_home_button_states()

    def set_source(self, source):
        # Selecting SOURCE starts a new mission selection.
        self.source = source
        self.destination = None
        self.mission_locked = False

        self.status_label.config(
            text=f"SOURCE = {self.source.upper()} — SELECT DESTINATION"
        )

        self.update_button_states()

    def set_destination(self, destination):
        # Destination cannot be changed after a mission is published.
        if self.source is None:
            self.status_label.config(
                text="SELECT SOURCE FIRST"
            )
            return

        if self.mission_locked:
            return

        self.destination = destination
        self.publish_mission()

    def publish_mission(self):
        self.ros_node.publish_mission(
            self.source,
            self.destination
        )

        self.mission_locked = True
        self.operation_type = "mission"

        self.status_label.config(
            text=(
                f"{self.source.upper()} → "
                f"{self.destination.upper()}"
            )
        )

        self.update_button_states()

    def update_home_button_states(self):
        if self.home == "top":
            self.home_top_button.config(
                relief=tk.SUNKEN,
                bg=self.selected_button_bg,
                fg=self.selected_button_fg
            )
            self.home_bottom_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
        elif self.home == "bottom":
            self.home_top_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
            self.home_bottom_button.config(
                relief=tk.SUNKEN,
                bg=self.selected_button_bg,
                fg=self.selected_button_fg
            )
        else:
            self.home_top_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
            self.home_bottom_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )

    def update_button_states(self):
        # SOURCE buttons are available whenever the robot is READY.
        self.source_top_button.config(state=tk.NORMAL)
        self.source_bottom_button.config(state=tk.NORMAL)

        if self.source == "top":
            self.source_top_button.config(
                relief=tk.SUNKEN,
                bg=self.selected_button_bg,
                fg=self.selected_button_fg
            )
            self.source_bottom_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
        elif self.source == "bottom":
            self.source_top_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
            self.source_bottom_button.config(
                relief=tk.SUNKEN,
                bg=self.selected_button_bg,
                fg=self.selected_button_fg
            )
        else:
            self.source_top_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
            self.source_bottom_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )

        # DESTINATION is available only after SOURCE is selected
        # and before the mission is published.
        if self.source is None or self.mission_locked:
            state = tk.DISABLED
        else:
            state = tk.NORMAL

        self.destination_top_button.config(state=state)
        self.destination_bottom_button.config(state=state)

        if self.destination == "top":
            self.destination_top_button.config(
                relief=tk.SUNKEN,
                bg=self.selected_button_bg,
                fg=self.selected_button_fg
            )
            self.destination_bottom_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
        elif self.destination == "bottom":
            self.destination_top_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
            self.destination_bottom_button.config(
                relief=tk.SUNKEN,
                bg=self.selected_button_bg,
                fg=self.selected_button_fg
            )
        else:
            self.destination_top_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )
            self.destination_bottom_button.config(
                relief=tk.RAISED,
                bg=self.default_button_bg,
                fg=self.default_button_fg,
            )



def main():
    rclpy.init()

    ros_node = RobotControlNode()

    root = tk.Tk()

    ui = RobotControlUI(root, ros_node)
    ros_node.set_ui(ui)

    def ros_spin():
        rclpy.spin_once(
            ros_node,
            timeout_sec=0.0
        )
        root.after(100, ros_spin)

    def shutdown():
        ros_node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", shutdown)

    root.after(100, ros_spin)
    root.mainloop()


if __name__ == "__main__":
    main()
