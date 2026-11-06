#!/usr/bin/env python3
# ==============================================================================
# SEZIONE 3.3 - CINEMATICA INVERSA (Spazio Cartesiano)
# NODO 1: Verifica di raggiungibilita'
#
# Descrizione:
#   Dato un punto cartesiano (X, Y, Z) espresso rispetto al frame 'base_link',
#   questo nodo verifica se il manipolatore UR3 puo' raggiungerlo.
#
#   NOTA DI PROGETTO (perche' compute_cartesian_path e non MoveGroup+OMPL):
#   Una prima versione di questo nodo usava l'azione /move_action con vincoli
#   di posizione/orientamento generici, pianificati con OMPL (RRTConnect).
#   Sui test reali questo ha causato due problemi distinti:
#     1. Con un orientamento assoluto fissato a priori -> traiettorie
#        "contorte" per raggiungere pose lontane da quell'orientamento;
#     2. Con l'orientamento lasciato completamente libero -> OMPL, essendo
#        un pianificatore campionante (non a percorso minimo), sceglieva
#        soluzioni IK anche molto diverse da quella attuale per spostamenti
#        di pochi centimetri (es. il polso che ruota di centinaia di gradi).
#   OMPL e' pensato per pianificazioni globali complesse, non per piccoli
#   spostamenti punto-punto: lo strumento corretto per questo caso e' invece
#   il servizio '/compute_cartesian_path', che INTERPOLA linearmente la posa
#   dal punto attuale al target e risolve la IK passo-passo lungo la retta.
#   Questo garantisce continuita' con la configurazione attuale (niente
#   salti di configurazione) e nessuna deviazione (niente detour assurdi).
#
#   Qui il servizio viene usato SENZA mai eseguire il movimento: serve solo a
#   calcolare quanta parte del percorso rettilineo verso il target sarebbe
#   effettivamente realizzabile (campo 'fraction' della risposta, 0.0-1.0),
#   tenendo conto di IK, limiti articolari e collisioni (avoid_collisions=True).
#   fraction ≈ 1.0 => il punto e' raggiungibile in linea retta senza ostacoli.
#
#   La classe e' pensata per essere sia eseguita come nodo standalone (vedi
#   main), sia importata e riutilizzata da altri nodi (vedi Nodo 3 -
#   08_interfaccia_continua.py) tramite importlib.import_module, dato che il
#   nome del file inizia con una cifra e non e' quindi importabile con una
#   normale istruzione "import".
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose


# Nome del gruppo di pianificazione definito nel MoveIt config dell'UR3
GRUPPO_PIANIFICAZIONE = 'ur_manipulator'
# Link che rappresenta la punta dell'end-effector
LINK_PUNTA = 'tool0'
# Frame rispetto al quale sono espresse le coordinate X, Y, Z
FRAME_RIFERIMENTO = 'base_link'

# Frazione minima del percorso rettilineo che deve essere calcolabile perche'
# il punto sia considerato raggiungibile (1.0 = 100% del tragitto verso il target)
FRAZIONE_MINIMA = 0.99


class VerificaRaggiungibilita(Node):
    """
    Nodo/componente che verifica la raggiungibilita' di una posa cartesiana
    calcolando (senza mai eseguirlo) il percorso lineare verso di essa.
    """

    def __init__(self, nome_nodo='nodo_verifica_raggiungibilita'):
        super().__init__(nome_nodo)

        # Client per il servizio di calcolo del percorso cartesiano
        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )

        # Buffer/listener TF, usati per leggere l'orientamento ATTUALE di
        # tool0 e usarlo come orientamento "di default" del waypoint target
        # (mantenerlo costante durante lo spostamento evita rotazioni del
        # polso non necessarie).
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info('Nodo di verifica raggiungibilita\' avviato.')
        self.get_logger().info('Attendo il servizio /compute_cartesian_path...')

        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '❌ Servizio /compute_cartesian_path non disponibile. '
                'Assicurati che MoveIt 2 (move_group) sia in esecuzione.'
            )
        else:
            self.get_logger().info('✅ Servizio /compute_cartesian_path connesso.')

    # --------------------------------------------------------------------
    # Lettura dell'orientamento attuale (via TF)
    # --------------------------------------------------------------------
    def _orientamento_attuale(self):
        """Legge via TF l'orientamento corrente di tool0 rispetto a
        base_link, da usare come orientamento del waypoint target quando non
        specificato esplicitamente (vedi nota in testa al file)."""
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self._tf_buffer.lookup_transform(
                    FRAME_RIFERIMENTO, LINK_PUNTA, Time()
                )
                return tf.transform.rotation
            except TransformException:
                continue

        self.get_logger().warn(
            'Impossibile leggere l\'orientamento attuale via TF: uso un '
            'orientamento di default (pinza verso il basso).'
        )
        default = Pose().orientation
        default.x, default.y, default.z, default.w = 1.0, 0.0, 0.0, 0.0
        return default

    # --------------------------------------------------------------------
    # Interpretazione del codice di errore restituito da MoveIt
    # --------------------------------------------------------------------
    @staticmethod
    def _interpreta_codice(codice):
        mappa = {
            MoveItErrorCodes.SUCCESS: '✅ Calcolo del percorso riuscito.',
            MoveItErrorCodes.NO_IK_SOLUTION: (
                '❌ Nessuna soluzione di cinematica inversa: la posa e\' fuori '
                'dallo spazio di lavoro raggiungibile oppure ricade in una '
                'configurazione singolare.'
            ),
            MoveItErrorCodes.GOAL_IN_COLLISION: '❌ La posa obiettivo comporta una collisione.',
            MoveItErrorCodes.START_STATE_IN_COLLISION: (
                '❌ La configurazione di partenza del robot e\' gia\' in collisione.'
            ),
            MoveItErrorCodes.PLANNING_FAILED: '❌ Calcolo del percorso fallito.',
            MoveItErrorCodes.TIMED_OUT: '❌ Tempo massimo di calcolo superato.',
            MoveItErrorCodes.INVALID_MOTION_PLAN: '❌ Piano di movimento non valido.',
        }
        return mappa.get(codice, f'❌ Calcolo fallito (codice MoveIt: {codice}).')

    # --------------------------------------------------------------------
    # Metodo pubblico principale
    # --------------------------------------------------------------------
    def verifica_posa(self, x, y, z, orientamento=None, max_step=0.01,
                       timeout=10.0):
        """
        Verifica se la posa (x, y, z) e' raggiungibile in linea retta dalla
        posizione attuale, senza collisioni ne' singolarita' cinematiche.

        Ritorna un dizionario:
            {
              'raggiungibile': bool,
              'codice': int,          # MoveItErrorCodes
              'frazione': float,      # 0.0-1.0, quota di percorso calcolata
              'messaggio': str,       # descrizione leggibile
            }
        """
        if orientamento is None:
            orientamento = self._orientamento_attuale()

        if not self._cartesian_client.service_is_ready():
            if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
                return {
                    'raggiungibile': False,
                    'codice': None,
                    'frazione': 0.0,
                    'messaggio': '❌ Servizio /compute_cartesian_path non disponibile.',
                }

        self.get_logger().info(
            f'Verifico raggiungibilita\' del punto '
            f'(X={x:.3f}, Y={y:.3f}, Z={z:.3f})...'
        )

        richiesta = GetCartesianPath.Request()
        richiesta.header.frame_id = FRAME_RIFERIMENTO
        richiesta.group_name = GRUPPO_PIANIFICAZIONE
        richiesta.link_name = LINK_PUNTA
        richiesta.start_state.is_diff = True

        waypoint = Pose()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoint.orientation = orientamento
        richiesta.waypoints = [waypoint]

        richiesta.max_step = max_step
        richiesta.jump_threshold = 0.0
        richiesta.avoid_collisions = True

        future = self._cartesian_client.call_async(richiesta)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        risposta = future.result()

        if risposta is None:
            messaggio = '❌ Nessuna risposta dal servizio /compute_cartesian_path (timeout).'
            self.get_logger().error(messaggio)
            return {'raggiungibile': False, 'codice': None, 'frazione': 0.0, 'messaggio': messaggio}

        codice = risposta.error_code.val
        frazione = risposta.fraction
        raggiungibile = (codice == MoveItErrorCodes.SUCCESS and frazione >= FRAZIONE_MINIMA)

        if raggiungibile:
            messaggio = '✅ Posa raggiungibile: percorso rettilineo completo (100%).'
        elif codice != MoveItErrorCodes.SUCCESS:
            messaggio = self._interpreta_codice(codice)
        else:
            messaggio = (
                f'❌ Percorso rettilineo incompleto: raggiunto solo il '
                f'{frazione * 100:.0f}% del tragitto (collisione o limite articolare '
                'lungo il percorso).'
            )

        self.get_logger().info(messaggio)

        return {
            'raggiungibile': raggiungibile,
            'codice': codice,
            'frazione': frazione,
            'messaggio': messaggio,
        }


# ==============================================================================
# Esecuzione standalone del nodo (per test/dimostrazione del solo Nodo 1)
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    nodo = VerificaRaggiungibilita()

    try:
        print('--- Verifica di raggiungibilita\' (standalone) ---')
        x = float(input('Coordinata X [m]: '))
        y = float(input('Coordinata Y [m]: '))
        z = float(input('Coordinata Z [m]: '))

        esito = nodo.verifica_posa(x, y, z)
        print(esito['messaggio'])
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
