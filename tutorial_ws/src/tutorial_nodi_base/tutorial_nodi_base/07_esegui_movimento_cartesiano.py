#!/usr/bin/env python3
# ==============================================================================
# SEZIONE 3.3 - CINEMATICA INVERSA (Spazio Cartesiano)
# NODO 2: Esecuzione della movimentazione
#
# Descrizione:
#   Riceve delle coordinate cartesiane (X, Y, Z) - tipicamente gia' validate
#   dal Nodo 1 (06_verifica_raggiungibilita.py) - e comanda al robot il
#   movimento verso quel punto.
#
#   Come nel Nodo 1, il movimento cartesiano NON usa piu' l'azione generica
#   /move_action con pianificazione OMPL (RRTConnect): quel percorso, per
#   piccoli spostamenti punto-punto, poteva produrre traiettorie con ampie
#   riconfigurazioni del polso (OMPL e' un pianificatore campionante, pensato
#   per problemi globali, non per seguire una linea retta). Al suo posto:
#     1. si calcola il percorso con '/compute_cartesian_path' (interpolazione
#        lineare della posa, IK risolta passo-passo con continuita' rispetto
#        alla configurazione attuale);
#     2. SOLO se il percorso e' completo (fraction >= 0.99) lo si esegue
#        davvero, inviandolo all'azione standard /execute_trajectory.
#   Se il percorso risulta incompleto il movimento NON viene eseguito, per
#   sicurezza (evita di far percorrere al robot solo "un pezzo" di traiettoria
#   e fermarsi a meta').
#
#   Prima dell'esecuzione, la traiettoria calcolata viene anche allineata sui
#   giri interi (vedi _allinea_giri_continui): il polso (wrist_3) puo' ruotare
#   di piu' giri completi, quindi la stessa posa fisica ha infinite soluzioni
#   IK sfasate di multipli di 2*pi. Sul robot reale, se il primo punto della
#   traiettoria calcolata non coincide (a meno di 2*pi) con la posizione
#   REALE letta dagli encoder, il controller rifiuta subito la traiettoria
#   per "state tolerance violation" (visto sui test reali: errore ≈ 6.283 rad
#   = 2*pi). In simulazione questo non emerge perche' Gazebo non ha giri
#   accumulati sui giunti.
#
#   FALLBACK MoveL -> MoveJ: un percorso rettilineo (MoveL) puo' risultare
#   incompleto anche se il punto di partenza e quello di arrivo sono
#   SINGOLARMENTE raggiungibili: la retta che li congiunge puo' uscire dallo
#   spazio di lavoro a meta' strada (tipico per spostamenti ampi, es. da
#   X=+0.3 a X=-0.3, passando vicino/attraverso la base). In quel caso, invece
#   di fallire, il nodo calcola la cinematica inversa (IK) del solo punto
#   finale usando la configurazione ATTUALE come seed (/compute_ik: il
#   risolutore numerico converge alla soluzione IK piu' vicina alla partenza,
#   niente salti arbitrari di configurazione) e raggiunge quei valori di
#   giunto con un moto a giunti diretto (MoveJ) - la stessa logica, gia'
#   collaudata e priva di contorsioni, usata per vai_a_home.
#
#   Il nodo espone inoltre un metodo per riportare il robot in una posa di
#   HOME predefinita nello spazio dei giunti (piu' robusto e ripetibile di un
#   punto cartesiano per un rientro di sicurezza): questa parte usa ancora
#   /move_action con un vincolo di giunto, perche' e' un moto pianificato
#   liberamente (non deve seguire una retta) e non soffre degli stessi problemi.
#
#   Come per il Nodo 1, la classe e' pensata per essere sia eseguita in modo
#   standalone (vedi main), sia importata e riutilizzata (vedi Nodo 3 -
#   08_interfaccia_continua.py) tramite importlib.import_module, dato che il
#   nome del file inizia con una cifra.
# ==============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
    MoveItErrorCodes,
)
from geometry_msgs.msg import Pose, PoseStamped


GRUPPO_PIANIFICAZIONE = 'ur_manipulator'
LINK_PUNTA = 'tool0'
FRAME_RIFERIMENTO = 'base_link'

# Frazione minima del percorso rettilineo richiesta per procedere all'esecuzione
FRAZIONE_MINIMA = 0.99

# Nomi dei 6 giunti dell'UR3, nell'ordine standard usato da MoveIt/ros2_control
NOMI_GIUNTI = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Posizione di HOME, misurata sul robot reale (Teach Pendant, in gradi) e
# convertita in radianti. Ordine: base, spalla, gomito, polso1, polso2, polso3
# (corrispondente a NOMI_GIUNTI).
HOME_JOINTS_GRADI = [-85.33, -91.97, 91.80, -89.71, -90.33, 94.68]
HOME_JOINTS = [math.radians(g) for g in HOME_JOINTS_GRADI]


class EsecuzioneMovimento(Node):
    """
    Nodo/componente che calcola ED ESEGUE un moto rettilineo verso una posa
    cartesiana, oppure riporta il robot alla posizione di HOME nello spazio
    dei giunti.
    """

    def __init__(self, nome_nodo='nodo_esecuzione_movimento'):
        super().__init__(nome_nodo)

        # Per il movimento cartesiano rettilineo (X, Y, Z)
        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )
        self._execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Per il rientro in HOME e per il fallback MoveJ (spazio dei giunti)
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')

        # Per calcolare la IK del fallback MoveJ
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Per leggere l'orientamento ATTUALE di tool0 (vedi Nodo 1)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Per leggere le posizioni REALI attuali dei giunti (allineamento giri,
        # vedi _allinea_giri_continui)
        self._joint_state_attuale = {}
        self._sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self._callback_joint_states, 10
        )

        self.get_logger().info('Nodo di esecuzione movimentazione avviato.')
        self.get_logger().info('Attendo i server MoveIt (move_group)...')

        ok_cartesiano = self._cartesian_client.wait_for_service(timeout_sec=10.0)
        ok_esecuzione = self._execute_client.wait_for_server(timeout_sec=10.0)
        ok_move_group = self._move_group_client.wait_for_server(timeout_sec=10.0)
        ok_ik = self._ik_client.wait_for_service(timeout_sec=10.0)

        if ok_cartesiano and ok_esecuzione and ok_move_group and ok_ik:
            self.get_logger().info('✅ Server MoveIt connessi.')
        else:
            self.get_logger().error(
                f'❌ Server mancanti - compute_cartesian_path: {ok_cartesiano}, '
                f'execute_trajectory: {ok_esecuzione}, move_action: {ok_move_group}, '
                f'compute_ik: {ok_ik}. Assicurati che MoveIt 2 (move_group) sia in esecuzione.'
            )

    def _callback_joint_states(self, msg):
        for nome, posizione in zip(msg.name, msg.position):
            self._joint_state_attuale[nome] = posizione

    # --------------------------------------------------------------------
    # Allineamento sui giri interi (2*pi) rispetto allo stato REALE del robot
    # --------------------------------------------------------------------
    def _allinea_giri_continui(self, traiettoria):
        """Sposta l'intera traiettoria di un multiplo di 2*pi, per ciascun
        giunto, in modo che il suo primo punto coincida (a meno della
        tolleranza normale) con la posizione REALE attuale letta da
        /joint_states. Necessario perche' i giunti a rotazione continua del
        polso (in particolare wrist_3) hanno infinite soluzioni IK fisicamente
        identiche ma sfasate di giri interi: senza questo allineamento, il
        controller reale rifiuta la traiettoria a causa di un errore di stato
        di ~2*pi rad (vedi nota in testa al file)."""

        # Assicura di avere una lettura recente di /joint_states
        for _ in range(20):
            if self._joint_state_attuale:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        nomi_giunti_traiettoria = traiettoria.joint_trajectory.joint_names
        punti = traiettoria.joint_trajectory.points

        if not punti or not self._joint_state_attuale:
            return traiettoria

        for indice, nome in enumerate(nomi_giunti_traiettoria):
            attuale = self._joint_state_attuale.get(nome)
            if attuale is None:
                continue

            pianificato_iniziale = punti[0].positions[indice]
            giri = round((pianificato_iniziale - attuale) / (2 * math.pi))

            if giri != 0:
                offset = giri * 2 * math.pi
                self.get_logger().info(
                    f'Allineo {nome}: sfasamento di {giri} giro/i (2*pi) '
                    'rispetto allo stato reale, correggo.'
                )
                for punto in punti:
                    posizioni = list(punto.positions)
                    posizioni[indice] -= offset
                    punto.positions = posizioni

        return traiettoria

    # --------------------------------------------------------------------
    # Lettura della posa attuale di tool0 (via TF)
    # --------------------------------------------------------------------
    def _leggi_tf_tool0(self):
        """Interroga TF per la trasformata base_link -> tool0. Ritorna
        l'oggetto TransformStamped, oppure None se non disponibile entro
        il timeout."""
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self._tf_buffer.lookup_transform(
                    FRAME_RIFERIMENTO, LINK_PUNTA, Time()
                )
            except TransformException:
                continue
        return None

    def _orientamento_attuale(self):
        """Orientamento (quaternione) attuale di tool0 rispetto a base_link."""
        tf = self._leggi_tf_tool0()
        if tf is not None:
            return tf.transform.rotation

        self.get_logger().warn(
            'Impossibile leggere l\'orientamento attuale via TF: uso un '
            'orientamento di default (pinza verso il basso).'
        )
        default = Pose().orientation
        default.x, default.y, default.z, default.w = 1.0, 0.0, 0.0, 0.0
        return default

    def posizione_attuale(self):
        """Posizione (x, y, z) attuale di tool0 rispetto a base_link, in
        metri. Ritorna None se la TF non e' disponibile. Metodo pubblico:
        usato dal Nodo 3 per calcolare il target assoluto dei comandi
        RELATIVI (dx, dy, dz) ricevuti via topic vocale."""
        tf = self._leggi_tf_tool0()
        if tf is None:
            self.get_logger().error('❌ Impossibile leggere la posizione attuale via TF.')
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    # --------------------------------------------------------------------
    # Rallentamento manuale di una traiettoria gia' calcolata
    # --------------------------------------------------------------------
    @staticmethod
    def _rallenta_traiettoria(traiettoria, fattore):
        """Dilata nel tempo una RobotTrajectory gia' calcolata da
        compute_cartesian_path (che la restituisce a velocita' piena),
        scalando di conseguenza velocita' e accelerazioni di ogni punto.

        'fattore' e' la frazione della velocita' originale da mantenere
        (es. 0.1 = 10 volte piu' lenta). Il tempo di ogni punto viene diviso
        per 'fattore' (dilatato), le velocita' moltiplicate per 'fattore',
        le accelerazioni per 'fattore' al quadrato (derivata seconda)."""
        for punto in traiettoria.joint_trajectory.points:
            t = punto.time_from_start.sec + punto.time_from_start.nanosec * 1e-9
            nuovo_t = t / fattore
            punto.time_from_start.sec = int(nuovo_t)
            punto.time_from_start.nanosec = int(round((nuovo_t - int(nuovo_t)) * 1e9))

            punto.velocities = [v * fattore for v in punto.velocities]
            punto.accelerations = [a * fattore * fattore for a in punto.accelerations]

        return traiettoria

    # --------------------------------------------------------------------
    # Calcolo IK per il fallback MoveJ (seed = configurazione attuale)
    # --------------------------------------------------------------------
    def _calcola_ik(self, x, y, z, orientamento, timeout=5.0):
        """Calcola la cinematica inversa per la posa (x, y, z, orientamento)
        usando la configurazione ATTUALE del robot come seed: il risolutore
        numerico di MoveIt converge alla soluzione IK piu' vicina alla
        partenza (continuita', nessun salto arbitrario di configurazione,
        a differenza della vecchia pianificazione OMPL generica).

        Ritorna la lista di posizioni di giunto (ordine NOMI_GIUNTI), oppure
        None se il calcolo fallisce."""
        if not self._ik_client.service_is_ready():
            if not self._ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('❌ Servizio /compute_ik non disponibile.')
                return None

        richiesta = GetPositionIK.Request()
        richiesta.ik_request.group_name = GRUPPO_PIANIFICAZIONE
        richiesta.ik_request.avoid_collisions = True
        # is_diff=True, senza joint_state esplicito: usa lo stato ATTUALE
        # del robot come seed per la IK (stessa convenzione usata altrove).
        richiesta.ik_request.robot_state.is_diff = True

        posa = PoseStamped()
        posa.header.frame_id = FRAME_RIFERIMENTO
        posa.pose.position.x = x
        posa.pose.position.y = y
        posa.pose.position.z = z
        posa.pose.orientation = orientamento
        richiesta.ik_request.pose_stamped = posa

        future = self._ik_client.call_async(richiesta)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        risposta = future.result()

        if risposta is None or risposta.error_code.val != MoveItErrorCodes.SUCCESS:
            codice = risposta.error_code.val if risposta is not None else None
            self.get_logger().error(f'❌ Calcolo IK fallito (codice MoveIt: {codice}).')
            return None

        mappa = dict(zip(
            risposta.solution.joint_state.name,
            risposta.solution.joint_state.position,
        ))
        try:
            return [mappa[nome] for nome in NOMI_GIUNTI]
        except KeyError as errore:
            self.get_logger().error(f'❌ Giunto mancante nella soluzione IK: {errore}')
            return None

    def _esegui_movimento_giunti_verso_posa(self, x, y, z, orientamento, timeout):
        """Fallback MoveJ: calcola la IK del target e la raggiunge con un
        moto diretto nello spazio dei giunti (vedi nota in testa al file)."""
        self.get_logger().warn(
            'Percorso rettilineo non praticabile: provo un movimento a '
            'giunti (MoveJ) verso lo stesso target...'
        )

        posizioni_giunti = self._calcola_ik(x, y, z, orientamento)
        if posizioni_giunti is None:
            messaggio = '❌ Impossibile calcolare una soluzione IK (MoveJ) per il target: movimento annullato.'
            self.get_logger().error(messaggio)
            return {'eseguito': False, 'codice': None, 'messaggio': messaggio}

        esito = self._pianifica_esegui_giunti(posizioni_giunti, timeout)
        if esito['eseguito']:
            esito['messaggio'] = (
                '✅ Movimento eseguito correttamente (MoveJ - il percorso '
                'rettilineo non era praticabile, usato un moto a giunti).'
            )
            self.get_logger().info(esito['messaggio'])
        return esito

    # --------------------------------------------------------------------
    # Movimento cartesiano rettilineo (X, Y, Z)
    # --------------------------------------------------------------------
    def esegui_movimento_cartesiano(self, x, y, z, orientamento=None,
                                     max_step=0.01, fattore_velocita=0.1,
                                     timeout=20.0):
        """Calcola il percorso rettilineo verso (x, y, z) e, solo se completo
        al 100%, lo esegue davvero sui controller reali/simulati (rallentato
        di 'fattore_velocita' rispetto al calcolo a velocita' piena).

        Se 'orientamento' non e' specificato, viene mantenuto quello ATTUALE
        del polso per tutta la traiettoria (nessuna rotazione superflua)."""

        if orientamento is None:
            orientamento = self._orientamento_attuale()

        self.get_logger().info(
            f'--- Calcolo percorso verso (X={x:.3f}, Y={y:.3f}, Z={z:.3f}) ---'
        )

        if not self._cartesian_client.service_is_ready():
            if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
                return {
                    'eseguito': False,
                    'codice': None,
                    'messaggio': '❌ Servizio /compute_cartesian_path non disponibile.',
                }

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
        # NOTA: in questa versione di moveit_msgs GetCartesianPath NON accetta
        # max_velocity_scaling_factor/max_acceleration_scaling_factor (a
        # differenza di MotionPlanRequest): il percorso viene quindi calcolato
        # a velocita' piena e rallentato manualmente dopo (vedi
        # _rallenta_traiettoria), prima di eseguirlo sul robot reale.

        future = self._cartesian_client.call_async(richiesta)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        risposta = future.result()

        if risposta is None:
            messaggio = '❌ Nessuna risposta dal servizio /compute_cartesian_path (timeout).'
            self.get_logger().error(messaggio)
            return {'eseguito': False, 'codice': None, 'messaggio': messaggio}

        if risposta.fraction < FRAZIONE_MINIMA or risposta.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f'Percorso rettilineo incompleto ({risposta.fraction * 100:.0f}%).'
            )
            # Fallback MoveL -> MoveJ: la retta non e' praticabile, ma il
            # punto finale potrebbe comunque essere raggiungibile con un
            # moto a giunti diretto (vedi nota in testa al file).
            return self._esegui_movimento_giunti_verso_posa(x, y, z, orientamento, timeout)

        traiettoria = self._allinea_giri_continui(risposta.solution)
        traiettoria = self._rallenta_traiettoria(traiettoria, fattore_velocita)

        self.get_logger().info(
            f'Percorso calcolato al 100%. Eseguo la traiettoria '
            f'(velocita\' scalata al {fattore_velocita * 100:.0f}%)...'
        )

        # -- Esecuzione della traiettoria calcolata --
        if not self._execute_client.server_is_ready():
            if not self._execute_client.wait_for_server(timeout_sec=5.0):
                return {
                    'eseguito': False,
                    'codice': None,
                    'messaggio': '❌ Server /execute_trajectory non disponibile.',
                }

        goal_esecuzione = ExecuteTrajectory.Goal()
        goal_esecuzione.trajectory = traiettoria

        future_exec = self._execute_client.send_goal_async(goal_esecuzione)
        rclpy.spin_until_future_complete(self, future_exec, timeout_sec=timeout)
        goal_handle = future_exec.result()

        if goal_handle is None or not goal_handle.accepted:
            messaggio = '❌ Richiesta di esecuzione rifiutata dal server.'
            self.get_logger().error(messaggio)
            return {'eseguito': False, 'codice': None, 'messaggio': messaggio}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result().result

        codice = result.error_code.val
        eseguito = (codice == MoveItErrorCodes.SUCCESS)
        messaggio = (
            '✅ Movimento eseguito correttamente (traiettoria rettilinea).' if eseguito
            else f'❌ Esecuzione fallita (codice MoveIt: {codice}).'
        )
        self.get_logger().info(messaggio)
        return {'eseguito': eseguito, 'codice': codice, 'messaggio': messaggio}

    # --------------------------------------------------------------------
    # Movimento a giunti (HOME e fallback MoveJ) - schema PIANIFICA, ALLINEA
    # I GIRI, POI ESEGUI: identico a quello usato per il moto cartesiano, per
    # evitare la "state tolerance violation" da 2*pi vista sul robot reale
    # (vedi nota in testa al file). In precedenza questo pathway pianificava
    # ED ESEGUIVA in un'unica chiamata /move_action (plan_only=False): comodo,
    # ma impediva di intercettare e allineare la traiettoria prima di
    # eseguirla, motivo per cui l'errore -4 poteva ripresentarsi anche qui.
    # --------------------------------------------------------------------
    def _crea_richiesta_giunti(self, posizioni_giunti):
        """Costruisce il goal MoveGroup per un target di giunto, in sola
        modalita' di pianificazione (plan_only=True): l'esecuzione avviene
        sempre separatamente, dopo l'allineamento dei giri (vedi
        _pianifica_esegui_giunti)."""
        goal_msg = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = GRUPPO_PIANIFICAZIONE
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        request.start_state.is_diff = True

        vincoli = Constraints()
        for nome, posizione in zip(NOMI_GIUNTI, posizioni_giunti):
            vincolo = JointConstraint()
            vincolo.joint_name = nome
            vincolo.position = posizione
            vincolo.tolerance_above = 0.01
            vincolo.tolerance_below = 0.01
            vincolo.weight = 1.0
            vincoli.joint_constraints.append(vincolo)
        request.goal_constraints.append(vincoli)

        planning_options = PlanningOptions()
        planning_options.plan_only = True

        goal_msg.request = request
        goal_msg.planning_options = planning_options
        return goal_msg

    def _pianifica_esegui_giunti(self, posizioni_giunti, timeout=20.0):
        """Pianifica (soltanto) un moto verso 'posizioni_giunti', allinea la
        traiettoria risultante sui giri interi rispetto allo stato reale del
        robot, e SOLO DOPO la esegue via /execute_trajectory."""
        if not self._move_group_client.server_is_ready():
            if not self._move_group_client.wait_for_server(timeout_sec=5.0):
                return {
                    'eseguito': False,
                    'codice': None,
                    'messaggio': '❌ Server MoveGroup non disponibile.',
                }

        goal = self._crea_richiesta_giunti(posizioni_giunti)

        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            messaggio = '❌ Richiesta di pianificazione rifiutata dal server MoveGroup.'
            self.get_logger().error(messaggio)
            return {'eseguito': False, 'codice': None, 'messaggio': messaggio}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            messaggio = f'❌ Pianificazione fallita (codice MoveIt: {result.error_code.val}).'
            self.get_logger().error(messaggio)
            return {'eseguito': False, 'codice': result.error_code.val, 'messaggio': messaggio}

        traiettoria = self._allinea_giri_continui(result.planned_trajectory)

        if not self._execute_client.server_is_ready():
            if not self._execute_client.wait_for_server(timeout_sec=5.0):
                return {
                    'eseguito': False,
                    'codice': None,
                    'messaggio': '❌ Server /execute_trajectory non disponibile.',
                }

        goal_esecuzione = ExecuteTrajectory.Goal()
        goal_esecuzione.trajectory = traiettoria

        future_exec = self._execute_client.send_goal_async(goal_esecuzione)
        rclpy.spin_until_future_complete(self, future_exec, timeout_sec=timeout)
        goal_handle_exec = future_exec.result()

        if goal_handle_exec is None or not goal_handle_exec.accepted:
            messaggio = '❌ Richiesta di esecuzione rifiutata dal server.'
            self.get_logger().error(messaggio)
            return {'eseguito': False, 'codice': None, 'messaggio': messaggio}

        result_future_exec = goal_handle_exec.get_result_async()
        rclpy.spin_until_future_complete(self, result_future_exec, timeout_sec=timeout)
        result_exec = result_future_exec.result().result

        codice = result_exec.error_code.val
        eseguito = (codice == MoveItErrorCodes.SUCCESS)
        messaggio = (
            '✅ Movimento eseguito correttamente.' if eseguito
            else f'❌ Esecuzione fallita (codice MoveIt: {codice}).'
        )
        self.get_logger().info(messaggio)
        return {'eseguito': eseguito, 'codice': codice, 'messaggio': messaggio}

    def vai_a_home(self, timeout=20.0):
        """Riporta il robot alla posizione di HOME nello spazio dei giunti."""
        self.get_logger().info('--- Rientro in posizione di HOME ---')
        return self._pianifica_esegui_giunti(HOME_JOINTS, timeout)


# ==============================================================================
# Esecuzione standalone del nodo (per test/dimostrazione del solo Nodo 2)
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    nodo = EsecuzioneMovimento()

    try:
        print('--- Esecuzione movimentazione (standalone) ---')
        print('Inserisci X, Y, Z oppure scrivi "home" per tornare in HOME.')
        comando = input('X (o "home"): ').strip().lower()

        if comando == 'home':
            esito = nodo.vai_a_home()
        else:
            x = float(comando)
            y = float(input('Coordinata Y [m]: '))
            z = float(input('Coordinata Z [m]: '))
            esito = nodo.esegui_movimento_cartesiano(x, y, z)

        print(esito['messaggio'])
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
