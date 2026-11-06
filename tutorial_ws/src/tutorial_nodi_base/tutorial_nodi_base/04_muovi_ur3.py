# ==============================================================================
# TUTORIAL 4: IL PRIMO MOVIMENTO (Inviare una Traiettoria)
# Descrizione: Questo nodo invia un comando diretto ai motori del robot in Gazebo o
#              al robot reale.
#              Qui diciamo a ogni singolo giunto:
#              "Vai a questi gradi in X secondi".
# ==============================================================================
 
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
class MuoviUR3(Node):
   def __init__(self):
       super().__init__('nodo_muovi_ur3')
       # Topic del controllore: in Gazebo '/joint_trajectory_controller/joint_trajectory',
       # sul robot reale '/scaled_joint_trajectory_controller/joint_trajectory'.
       # Selezionabile senza modificare il codice:
       #   --ros-args -p topic_traiettoria:=/scaled_joint_trajectory_controller/joint_trajectory
       self.declare_parameter('topic_traiettoria', '/joint_trajectory_controller/joint_trajectory')
       topic = self.get_parameter('topic_traiettoria').get_parameter_value().string_value
       self.publisher_ = self.create_publisher(JointTrajectory, topic, 10)
       self.timer = self.create_timer(6.0, self.esegui_movimento)
       self.get_logger().info('Nodo avviato. Preparazione del movimento in corso...')
   def esegui_movimento(self):
       msg = JointTrajectory()
       msg.header.stamp.sec = 0             # fondamentale per il funzionamento del robot reale   
       msg.header.stamp.nanosec = 0         # fondamentale per il funzionamento del robot reale
       msg.joint_names = [
           'shoulder_pan_joint',
           'shoulder_lift_joint',
           'elbow_joint',
           'wrist_1_joint',
           'wrist_2_joint',
           'wrist_3_joint'
       ]
       punto = JointTrajectoryPoint()
       punto.positions = [-1.0629, -1.6933, 1.3547, -1.5230, -1.6013, 0.0827]       # posizione sicura
       punto.time_from_start = Duration(sec=5, nanosec=0)
       msg.points.append(punto)
       self.publisher_.publish(msg)
       self.get_logger().info('Comando inviato!')
       self.timer.cancel()
def main(args=None):
   rclpy.init(args=args)
   nodo = MuoviUR3()
   try:
       rclpy.spin(nodo)
   except KeyboardInterrupt:
       pass
   nodo.destroy_node()
   rclpy.shutdown()
if __name__ == '__main__':
   main()
