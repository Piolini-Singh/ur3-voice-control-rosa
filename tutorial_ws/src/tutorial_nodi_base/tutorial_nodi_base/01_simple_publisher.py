#!/usr/bin/env python3

# ==============================================================================

# TUTORIAL 1: IL PRIMO PUBLISHER

# Descrizione: Questo nodo crea un trasmettitore base che invia un messaggio

#              di testo continuo sulla rete ROS 2n sul canale di INPUT.

# ==============================================================================
 
# 1. Gli Import

import rclpy

from rclpy.node import Node

from std_msgs.msg import String
 
# 2. La Creazione del tuo Nodo Personalizzato

class InputPub(Node):

    def __init__(self):

        super().__init__('nodo_input_pub')

        # 3. Creiamo il trasmettitore. Richiede tre parametri: Tipo (String), Canale ('input'), Coda (10)

        self.publisher_ = self.create_publisher(String, 'input', 10)

        # Aggiungiamo un timer che fa scattare una funzione ogni secondo

        timer_period = 1.0  # secondi

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.contatore = 0  # Un semplice numero per contare i messaggi
 
    # Questa è la funzione che viene eseguita dal timer ogni secondo

    def timer_callback(self):

        # 4. La Preparazione del Pacchetto

        msg = String()                             # Prendo un pacchetto vuoto
        richiesta = f"Ciao mondo ROS 2! Messaggio n: {self.contatore}"

        msg.data = richiesta                       # Inserisco il testo nella busta

        # Spedisco il pacchetto al megafono

        self.publisher_.publish(msg)

        # Stampo a schermo quello che sto inviando (per far vedere all'utente che funziona)

        self.get_logger().info(f'Sto pubblicando: "{msg.data}"')

        self.contatore += 1
 
# 5. L'Avvio del Sistema (Il Main)

def main(args=None):

    # Accendiamo le comunicazioni ROS

    rclpy.init(args=args)

    # Creiamo un'istanza del nostro nodo

    nodo = InputPub()

    # Mettiamo il nodo in loop infinito, in ascolto e in azione

    try:

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        # Se l'utente preme Ctrl+C nel terminale, usciamo in modo pulito

        pass

    # Distruggiamo il nodo e spegniamo le comunicazioni in uscita

    nodo.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 
