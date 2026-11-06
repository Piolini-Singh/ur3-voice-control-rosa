#!/usr/bin/env python3

# ==============================================================================

# TUTORIAL 5: LO SMISTATORE (Logica di Controllo Completa)

# Descrizione: Ascolta i comandi di testo dell'utente ("1" o "2")

#              e, in base alla scelta, invia al robot la traiettoria

#              per raggiungere una posa pre-salvata.

# ==============================================================================
 
# 1. Le Fondamenta (Gli Import)

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from builtin_interfaces.msg import Duration
 
# 2. La Creazione del Nodo

class SmistatoreUR3(Node):

    def __init__(self):

        super().__init__('nodo_smistatore')
 
        # 3. IL MUSCOLO (Publisher): Parla con i motori di Gazebo

        # Topic del controllore: sul robot reale '/scaled_joint_trajectory_controller/joint_trajectory',
        # in Gazebo '/joint_trajectory_controller/joint_trajectory'. Selezionabile senza
        # modificare il codice:
        #   --ros-args -p topic_traiettoria:=/joint_trajectory_controller/joint_trajectory
        self.declare_parameter(
            'topic_traiettoria', '/scaled_joint_trajectory_controller/joint_trajectory'
        )
        topic = self.get_parameter('topic_traiettoria').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(JointTrajectory, topic, 10)
 
        # 4. L'ORECCHIO (Subscriber): Ascolta i tuoi comandi

        self.subscriber_ = self.create_subscription(

            String,

            '/input',

            self.ascolta_comandi,

            10

        )
 
        # 5. LA MEMORIA: Salviamo le posizioni in modo permanente nel codice

        # (Valori trovati in precedenza con il Coordinatore)

        self.pos_riposo = [-1.0629, -1.6933, 1.3547, -1.5230, -1.6013, 0.0827]     # Posizione destra '1'

        self.pos_lavoro = [-2.40, -1.75, 1.56, -1.89, -1.30, -1.57]   # Posizione sinistra '2'
 
        self.get_logger().info('Sistema Smistatore AVVIATO!')

        self.get_logger().info('Usa il topic /input: Scrivi "1" (Riposo) o "2" (Lavoro).')
 
    # 6. Callback di Ascolto (Il Cervello)

    def ascolta_comandi(self, msg):

        """Questa callback si attiva istantaneamente quando scrivi un comando."""

        if msg.data == '1':

            self.get_logger().info('Ordine ricevuto: Vado in Riposo...')

            self.invia_traiettoria(self.pos_riposo)

        elif msg.data == '2':

            self.get_logger().info('Ordine ricevuto: Vado al Lavoro...')

            self.invia_traiettoria(self.pos_lavoro)

        else:

            self.get_logger().info('Comando sconosciuto. Usa "1" o "2".')
 
    # 7. La Funzione di Movimento (L'Azione)

    def invia_traiettoria(self, angoli_destinazione):

        """Questa funzione prepara il fascicolo per Gazebo e lo spedisce."""

        msg = JointTrajectory()

        # Esecuzione immediata senza guardare l'orologio di Gazebo

        msg.header.stamp.sec = 0

        msg.header.stamp.nanosec = 0
 
        # Elenco dei giunti coinvolti

        msg.joint_names = [

            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',

            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'

        ]
 
        # Creazione della singola tappa (il foglietto)

        punto = JointTrajectoryPoint()

        punto.positions = angoli_destinazione

        punto.time_from_start = Duration(sec=5, nanosec=0) # Tempo di esecuzione: 3 secondi
 
        # Inseriamo il foglietto nel fascicolo e pubblichiamo

        msg.points.append(punto)

        self.publisher_.publish(msg)
 
# 8. L'Avvio del Sistema

def main(args=None):

    rclpy.init(args=args)

    nodo = SmistatoreUR3()

    try:

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        pass

    nodo.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 
