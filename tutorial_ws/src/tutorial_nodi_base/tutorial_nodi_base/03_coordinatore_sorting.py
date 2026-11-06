#!/usr/bin/env python3

# ==============================================================================

# TUTORIAL 3: IL COORDINATORE (Ascolto Multiplo e Logica)

# Descrizione: Questo nodo ascolta due canali contemporaneamente. 

#              1. Legge continuamente la posizione del robot da Gazebo.

#              2. Ascolta i tuoi comandi di testo. Se riceve la parola "punto",

#                 cattura la posizione in quell'istante e te la stampa a schermo.

# ==============================================================================
 
# 1. Le Fondamenta (Gli Import)

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState  # Per leggere i motori

from std_msgs.msg import String         # Per leggere i nostri comandi di testo
 
# 2. La Creazione del Nodo

class CoordinatoreSorting(Node):

    def __init__(self):

        super().__init__('nodo_coordinatore')

        # Variabile per memorizzare l'ultima posizione conosciuta del robot

        self.ultima_posizione = []

        # 3. Primo Orecchio: Sottoscrizione allo stato del robot

        self.sub_robot = self.create_subscription(

            JointState,

            '/joint_states',

            self.callback_robot,

            10

        )

        # 4. Secondo Orecchio: Sottoscrizione ai nostri comandi (es. dal terminale)

        self.sub_input = self.create_subscription(

            String,

            '/input',

            self.callback_input,

            10

        )

        self.get_logger().info('Coordinatore avviato. In attesa del comando "punto"...')
 
    # 5. Callback 1: Aggiorna continuamente la posizione

    def callback_robot(self, msg):

        # Salviamo la posizione attuale dei 6 giunti (arrotondata a 3 decimali)

        self.ultima_posizione = [round(p, 3) for p in msg.position]
 
    # 6. Callback 2: Scatta solo quando scriviamo qualcosa sul canale '/input'

    def callback_input(self, msg):

        # Controlliamo se la parola ricevuta è esattamente 'punto'

        if msg.data == 'punto':

            # Se è così, stampiamo la posizione salvata nell'altra callback!

            self.get_logger().info(f'📍 POSIZIONE CATTURATA: {self.ultima_posizione}')

            self.get_logger().info('Usa questi valori per programmare il movimento in C++ o Python!')
 
# 7. L'Avvio del Sistema

def main(args=None):

    rclpy.init(args=args)

    nodo = CoordinatoreSorting()

    try:

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        pass

    nodo.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 
