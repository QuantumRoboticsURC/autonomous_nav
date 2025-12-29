import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Int8, Float64MultiArray, Bool
from enum import Enum, auto
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

MOTION_STOP = 0
MOTION_POINTTOPOINT = 1
MOTION_CENTERANDAPPROACH = 2
MOTION_SEARCH = 3

class State(Enum):
    IDLE = auto()                
    ONLY_P2P = auto()            
    P2P_AND_SEARCH_GOTO = auto()
    SEARCH_ARUCO = auto()        
    CENTER_AND_APPROACH = auto() 
    DONE = auto()                


class SimpleStateMachine(Node):
    def __init__(self):
        super().__init__("state_machine")
        self.get_logger().info("Simple FSM node started")

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pub_state = self.create_publisher(Int8, "/state_command", 1)
        self.pub_target = self.create_publisher(Point, "/target_point", latching_qos)

        self.create_subscription(Int8, "/input_state", self.callback_input_state, 10)
        self.create_subscription(Float64MultiArray, "input_target", self.callback_input_target, 10)
        self.create_subscription(Bool, "/nav2_arrived", self.callback_arrived, 10)
        self.create_subscription(Bool, "/detect_aruco", self.callback_aruco, 10)
        self.create_subscription(Bool, "/done_aruco", self.callback_centered, 10)

        self.create_timer(0.05, self.timer_callback)

        self.hl_state = State.IDLE

        self.command_mode = None  

        self.current_target = Point()

        self.arrived = False
        self.aruco_detected = False
        self.aruco_done = False


    def send_motion_state(self, state_int: int):
        msg = Int8()
        msg.data = state_int
        self.pub_state.publish(msg)
        self.get_logger().info(f"[FSM] Publicando /state_command = {state_int}")

    def send_target_point(self):
        if self.current_target is None:
            self.get_logger().warn("[FSM] No hay target para enviar a /target_point")
            return
        self.pub_target.publish(self.current_target)
        self.get_logger().info(f"[FSM] Publicando /target_point = " f"({self.current_target.x:.6f}, {self.current_target.y:.6f})")

    def reset_flags(self):
        self.arrived = False
        self.aruco_detected = False
        self.aruco_done = False

    # Callbacks

    def callback_input_state(self, msg: Int8):
    
        self.command_mode = msg.data

        self.hl_state = State.IDLE
        self.reset_flags()

        self.get_logger().info(f"[FSM] Nuevo /input_state recibido: {self.command_mode}")

        if self.command_mode not in (0, 1):
            self.get_logger().warn("[FSM] input_state desconocido. Enviando STOP.")
            self.send_motion_state(MOTION_STOP)

    def callback_input_target(self, msg: Float64MultiArray):
 
        if len(msg.data) < 2:
            self.get_logger().warn("[FSM] input_target con menos de 2 valores.")
            return

        self.current_target.x = msg.data[0]
        self.current_target.y = msg.data[1]
       
      
        self.get_logger().info(f"[FSM] Nuevo target recibido en input_target: " f"({self.current_target.x:.6f}, {self.current_target.y:.6f})")

    def callback_arrived(self, msg: Bool):
        self.arrived = msg.data
        if self.arrived:
            self.get_logger().info("[FSM] Arrived=True recibido.")

    def callback_aruco(self, msg: Bool):
        self.aruco_detected = msg.data

    def callback_centered(self, msg: Bool):
        self.aruco_done = msg.data


    def timer_callback(self):
        if self.command_mode is None:
            return

        if self.command_mode == 0:
            if self.hl_state == State.IDLE:
                if self.current_target is not None:
                    self.get_logger().info("[FSM] Modo 0: iniciando ONLY_P2P")
                    self.send_target_point()
                    self.send_motion_state(MOTION_POINTTOPOINT)
                    self.hl_state = State.ONLY_P2P
                else:
                    self.get_logger().debug("[FSM] Modo 0: esperando target...")

            elif self.hl_state == State.ONLY_P2P:
                if self.arrived:
                    self.get_logger().info("[FSM] Modo 0: arrived=True, enviando STOP")
                    self.send_motion_state(MOTION_STOP)
                    self.hl_state = State.DONE

            elif self.hl_state == State.DONE:
                self.send_motion_state(MOTION_STOP)

        elif self.command_mode == 1:
            if self.hl_state == State.IDLE:
                if self.current_target is not None:
                    self.get_logger().info("[FSM] Modo 1: iniciando P2P_AND_SEARCH_GOTO")
                    self.send_target_point()
                    self.send_motion_state(MOTION_POINTTOPOINT)
                    self.hl_state = State.P2P_AND_SEARCH_GOTO
                else:
                    self.get_logger().debug("[FSM] Modo 1: esperando target...")

            elif self.hl_state == State.P2P_AND_SEARCH_GOTO:

                if self.aruco_detected:
                        self.get_logger().info("[FSM] Modo 1: ArUco detectado, saltando a CENTER_AND_APPROACH")
                        self.send_motion_state(MOTION_CENTERANDAPPROACH)
                        self.hl_state = State.CENTER_AND_APPROACH

                if self.arrived:
                    self.get_logger().info("[FSM] Modo 1: arrived=True, cambiando a SEARCH_ARUCO")
                    self.reset_flags() 
                    self.send_motion_state(MOTION_SEARCH)
                    self.hl_state = State.SEARCH_ARUCO

            elif self.hl_state == State.SEARCH_ARUCO:

                if self.aruco_detected:
                    self.get_logger().info("[FSM] Modo 1: ArUco detectado, cambiando a CENTER_AND_APPROACH")
                    self.aruco_detected = False  # consumir evento
                    self.send_motion_state(MOTION_CENTERANDAPPROACH)
                    self.hl_state = State.CENTER_AND_APPROACH

            elif self.hl_state == State.CENTER_AND_APPROACH:

                if self.aruco_done:
                    self.get_logger().info("[FSM] Modo 1: done_aruco=True, enviando STOP")
                    self.send_motion_state(MOTION_STOP)
                    self.hl_state = State.DONE

            elif self.hl_state == State.DONE:
                self.send_motion_state(MOTION_STOP)
            

        else:
            self.get_logger().warn(f"[FSM] command_mode desconocido ({self.command_mode}), enviando STOP.")
            self.send_motion_state(MOTION_STOP)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
