#!/usr/bin/env python3

# ==============================================================================

# TUTORIAL 2: IL PRIMO SUBSCRIBER (L'Orecchio)

# Descrizione: Questo nodo si mette in ascolto sul canale '/joint_states'.

#              Serve per leggere in tempo reale la posizione dei motori del robot

#              (in questo caso da Gazebo o dal robot vero) e stamparla a schermo.

# ==============================================================================
 
# 1. Le Fondamenta (Gli Import)

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState  # La "busta" specifica per i dati dei giunti
 
# 2. La Creazione del tuo Nodo Personalizzato

class AscoltatoreRobot(Node):

    def __init__(self):

        # Diamo ufficialmente un nome di battesimo al nostro nodo

        super().__init__('nodo_ascoltatore_ur3')

        # 3. La Creazione della Sottoscrizione (L'orecchio)

        # Richiede: Tipo messaggio (JointState), Canale ('/joint_states'), Callback, Coda (10)

        self.subscription = self.create_subscription(

            JointState,

            '/joint_states',

            self.elabora_dati,

            10

        )

        self.get_logger().info('Nodo in ascolto... Muovi il robot in Gazebo per vedere i dati!')
 
    # 4. La Callback (Cosa fare quando arriva un messaggio)

    # Questa funzione scatta in automatico OGNI VOLTA che Gazebo pubblica un nuovo stato

    def elabora_dati(self, msg):

        # Garbelli suggerisce di arrotondare i valori per leggibilità (ottimo consiglio!)

        # I valori originali sono in radianti con moltissimi decimali.

        posizioni = [round(p, 3) for p in msg.position]

        # Stampiamo solo i primi giunti per non intasare il terminale.

        # NOTA: l'ordine nell'array dipende da come Gazebo elenca i giunti.

        spalla = posizioni[0]

        gomito = posizioni[2]

        # Mostriamo i dati puliti a schermo

        self.get_logger().info(f'Ricevuto! Spalla: {spalla} rad | Gomito: {gomito} rad')
 
# 5. L'Avvio del Sistema (Il Main)

def main(args=None):

    # Accendiamo le comunicazioni ROS

    rclpy.init(args=args)

    # Creiamo un'istanza del nostro nodo

    nodo = AscoltatoreRobot()

    # Mettiamo il nodo in loop infinito, in ascolto (spin)

    try:

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        # Uscita pulita se l'utente preme Ctrl+C

        pass

    # Spegniamo tutto

    nodo.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 
