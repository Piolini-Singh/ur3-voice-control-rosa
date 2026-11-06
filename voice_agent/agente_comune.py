#!/usr/bin/env python3
# ==============================================================================
# MODULO CONDIVISO - Agente ROSA (tool + prompt + pubblicazione su ROS 2)
#
# Contiene tutto cio' che NON dipende dalla sorgente del comando (voce o
# tastiera): gli strumenti @tool che ROSA/GPT-4o puo' chiamare, il prompt di
# sistema, e l'inizializzazione del nodo ROS 2 + publisher su /comando_vocale.
#
# Usato da:
#   - 09_agente_vocale.py       (input vocale: microfono + Whisper)
#   - 09_agente_vocale_testo.py (input da tastiera, per test rapidi senza audio)
# ==============================================================================

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from langchain_openai import ChatOpenAI
from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts

# Stesso topic gia' ascoltato dal Nodo 3 (08_interfaccia_continua.py) in
# modalita' 'vocale' - NESSUNA modifica a quel nodo e' necessaria.
TOPIC_COMANDO_VOCALE = '/comando_vocale'

# File con la chiave OPENAI_API_KEY.
# Nella tesi il percorso era fisso ('/home/ur-robot/tirocinio_tesi/.env'); qui
# viene cercato nella radice del repository (accanto a .env.example), oppure nel
# percorso indicato dalla variabile d'ambiente UR3_VOICE_ENV_FILE. Se la
# variabile OPENAI_API_KEY e' gia' esportata nella shell, il file non serve.
PERCORSO_ENV = os.getenv(
    'UR3_VOICE_ENV_FILE',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, '.env'),
)

# Modello linguistico usato da ROSA (default della tesi: gpt-4o).
MODELLO_LLM = os.getenv('UR3_VOICE_LLM_MODEL', 'gpt-4o')


# ==============================================================================
# Stato globale condiviso con le funzioni @tool (funzioni "libere", non metodi
# di classe: e' il pattern usato anche negli esempi ufficiali di ROSA).
# ==============================================================================
_nodo_ros = None
_publisher_comando = None
richiesta_fine = False


def _pubblica_comando(comando: dict):
    """Serializza il dizionario in JSON e lo pubblica su /comando_vocale."""
    msg = String()
    msg.data = json.dumps(comando)
    _publisher_comando.publish(msg)
    rclpy.spin_once(_nodo_ros, timeout_sec=0.1)


# ==============================================================================
# Tool per ROSA: ognuno corrisponde esattamente a un "mode" del JSON gia'
# gestito dal Nodo 3 esistente.
# ==============================================================================
@tool
def muovi_assoluto(x: float, y: float, z: float) -> str:
    """
    Muove il manipolatore UR3 verso una posizione ASSOLUTA nello spazio
    cartesiano, in metri, rispetto al frame fisso 'base_link'.
    Usa questo strumento quando l'operatore indica coordinate precise
    (es. "vai a X=0.3 Y=0.2 Z=0.4").

    :param x: Coordinata X assoluta in metri.
    :param y: Coordinata Y assoluta in metri.
    :param z: Coordinata Z assoluta in metri.
    """
    _pubblica_comando({'mode': 'absolute', 'x': x, 'y': y, 'z': z})
    return f'Comando di movimento assoluto inviato: X={x}, Y={y}, Z={z}.'


@tool
def muovi_relativo(dx: float, dy: float, dz: float) -> str:
    """
    Sposta il manipolatore UR3 rispetto alla sua posizione ATTUALE, di un
    delta (dx, dy, dz) in metri. Usa questo strumento per comandi relativi
    (es. "alzati di 10 cm", "spostati a destra di 5 cm").

    :param dx: Spostamento relativo lungo X in metri.
    :param dy: Spostamento relativo lungo Y in metri.
    :param dz: Spostamento relativo lungo Z in metri (positivo = verso l'alto).
    """
    _pubblica_comando({'mode': 'relative', 'dx': dx, 'dy': dy, 'dz': dz})
    return f'Comando di movimento relativo inviato: dx={dx}, dy={dy}, dz={dz}.'


@tool
def vai_in_home() -> str:
    """
    Riporta il manipolatore UR3 nella posizione di HOME predefinita.
    Usa questo strumento quando l'operatore chiede di tornare a casa, in
    posizione di riposo, o di default.
    """
    _pubblica_comando({'mode': 'home'})
    return 'Comando di rientro in HOME inviato.'


@tool
def termina_sessione() -> str:
    """
    Termina la sessione di controllo e riporta il robot in HOME.
    Usa questo strumento SOLO quando l'operatore chiede esplicitamente di
    terminare, fermare o chiudere il controllo (es. "fine", "basta").
    """
    global richiesta_fine
    _pubblica_comando({'mode': 'fine'})
    richiesta_fine = True
    return 'Sessione terminata: il robot torna in HOME.'


STRUMENTI = [muovi_assoluto, muovi_relativo, vai_in_home, termina_sessione]


# ==============================================================================
# Prompt di sistema per ROSA
# ==============================================================================
PROMPTS_ROBOT = RobotSystemPrompts(
    embodiment_and_persona=(
        "Sei l'assistente vocale di un cobot Universal Robots UR3, "
        "controllato tramite ROS 2 Humble e MoveIt 2. Il tuo compito e' "
        "interpretare comandi in italiano e tradurli in movimenti del "
        "robot, chiamando lo strumento corretto."
    ),
    about_your_environment=(
        "Le coordinate cartesiane (x, y, z) sono espresse in metri rispetto "
        "al frame fisso 'base_link', l'origine alla base del robot.\n"
        "\n"
        "CONVENZIONE DIRENZIONALE (confermata empiricamente sul robot reale, "
        "dal punto di vista di un operatore che sta IN PIEDI DI FRONTE al "
        "robot): "
        "'destra' = +X, 'sinistra' = -X, 'indietro' = +Y, 'avanti' = -Y, "
        "'su'/'alzati' = +Z, 'giu'/'scendi' = -Z. "
        "Usa SEMPRE e SOLO questa convenzione per tradurre destra/sinistra/"
        "avanti/indietro/su/giu' in dx, dy, dz: non dedurla autonomamente."
    ),
    critical_instructions=(
        "Usa muovi_assoluto quando l'operatore indica coordinate X, Y, Z "
        "precise. Usa muovi_relativo quando chiede uno spostamento relativo "
        "rispetto alla posizione attuale (es. 'alzati di 10 cm', 'spostati "
        "a destra di 5 cm'). Usa vai_in_home quando chiede di tornare a "
        "casa o in posizione di riposo. Usa termina_sessione SOLO quando "
        "chiede esplicitamente di terminare/fermare il controllo.\n"
        "\n"
        "COMANDI COMPOSTI (piu' movimenti in un'unica frase): se l'operatore "
        "chiede piu' spostamenti in sequenza (es. 'vai a destra di 10 cm e "
        "poi alzati di 5 cm', 'sali un po' e spostati a sinistra'), chiama "
        "PIU' VOLTE lo strumento adatto, uno per ogni spostamento richiesto, "
        "NELL'ORDINE in cui sono stati detti. Non provare a sommare i "
        "movimenti in un'unica chiamata: ogni chiamata a muovi_relativo deve "
        "corrispondere a un solo spostamento nella frase.\n"
        "\n"
        "MAGNITUDINI VAGHE: se l'operatore non specifica un valore numerico "
        "preciso, usa queste equivalenze come default (in metri): "
        "'un pochino'/'leggermente' = 0.02, 'poco' = 0.05, 'un po' = 0.05, "
        "'molto'/'tanto' = 0.15, 'tantissimo' = 0.25. Se non riesci comunque "
        "a stimare una magnitudine ragionevole, chiedi all'operatore di "
        "specificare una distanza.\n"
        "\n"
        "REGOLA DI SICUREZZA: se il comando e' ambiguo, incompleto, o "
        "potenzialmente pericoloso (es. valori molto grandi, direzioni "
        "contraddittorie nella stessa frase), NON eseguire alcun movimento: "
        "fermati e chiedi chiarimenti all'operatore invece di indovinare i "
        "parametri."
    ),
)


# ==============================================================================
# Inizializzazione condivisa: nodo ROS 2 + publisher + agente ROSA
# ==============================================================================
def inizializza_agente():
    """Carica la chiave OpenAI, inizializza rclpy/publisher e costruisce
    l'agente ROSA con i tool e i prompt condivisi. Ritorna l'istanza ROSA
    pronta all'uso (agente.invoke(testo))."""
    global _nodo_ros, _publisher_comando

    from dotenv import load_dotenv
    load_dotenv(PERCORSO_ENV)
    if not os.getenv('OPENAI_API_KEY'):
        raise RuntimeError(
            f'OPENAI_API_KEY non trovata (ne\' nell\'ambiente ne\' in {PERCORSO_ENV}). '
            'Copia .env.example in .env e inserisci la tua chiave.'
        )

    rclpy.init()
    _nodo_ros = Node('nodo_agente_vocale')
    _publisher_comando = _nodo_ros.create_publisher(String, TOPIC_COMANDO_VOCALE, 10)

    # Attende che il subscriber del Nodo 3 sia effettivamente connesso, prima
    # di considerarsi pronti. Senza questa attesa, un publish troppo precoce
    # (subito dopo l'avvio) puo' andare perso a causa dei tempi di discovery
    # DDS: publish() non solleva mai un errore anche se nessuno e' in
    # ascolto, quindi il comando sparirebbe silenziosamente.
    print(f'In attesa che qualcuno sia in ascolto su {TOPIC_COMANDO_VOCALE}...')
    for _ in range(50):
        rclpy.spin_once(_nodo_ros, timeout_sec=0.1)
        if _publisher_comando.get_subscription_count() > 0:
            print('✅ Subscriber rilevato (Nodo 3 in ascolto).')
            break
    else:
        print(
            f'⚠️  Nessun subscriber rilevato su {TOPIC_COMANDO_VOCALE} dopo 5s: '
            'assicurati che il Nodo 3 sia gia\' avviato con '
            '"--ros-args -p modalita:=vocale" PRIMA di questo agente. '
            'Procedo comunque, ma i comandi rischiano di andare persi.'
        )

    llm = ChatOpenAI(model_name=MODELLO_LLM, temperature=0)

    agente = ROSA(
        ros_version=2,
        llm=llm,
        tools=STRUMENTI,
        prompts=PROMPTS_ROBOT,
    )
    return agente


def chiudi():
    """Distrugge il nodo ROS 2 e chiude rclpy in modo pulito."""
    if _nodo_ros is not None:
        _nodo_ros.destroy_node()
    rclpy.shutdown()
