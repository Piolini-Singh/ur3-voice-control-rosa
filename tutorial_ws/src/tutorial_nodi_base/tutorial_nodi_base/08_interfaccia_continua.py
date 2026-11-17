#!/usr/bin/env python3
# ==============================================================================
# SEZIONE 3.3 - CINEMATICA INVERSA (Spazio Cartesiano)
# NODO 3: Logiche di continuita' (interfaccia da tastiera E da topic vocale)
#
# Descrizione:
#   Nodo "orchestratore" che riutilizza il Nodo 1 (verifica raggiungibilita')
#   e il Nodo 2 (esecuzione movimentazione) come componenti. Supporta due
#   modalita' di ascolto, scelte con il parametro ROS 'modalita':
#
#     - modalita:=tastiera (default): l'utente digita tre coordinate "X Y Z"
#       nel terminale; digitando "fine" il ciclo termina.
#     - modalita:=vocale: il nodo resta in ascolto sul topic '/comando_vocale'
#       (std_msgs/String, payload JSON), pensato per essere alimentato da un
#       nodo esterno (agente vocale con Whisper + GPT-4o). Il ciclo termina
#       ricevendo {"mode": "fine"} oppure con Ctrl+C.
#
#   In ENTRAMBE le modalita' ogni richiesta di movimento passa sempre prima
#   dal Nodo 1 (verifica raggiungibilita') e solo se approvata dal Nodo 2
#   (esecuzione): la logica e' identica, cambia solo la sorgente del comando.
#
#   FORMATO JSON atteso sul topic '/comando_vocale':
#     Assoluto:  {"mode": "absolute", "x": 0.30, "y": -0.10, "z": 0.40}
#     Relativo:  {"mode": "relative", "dx": 0.05, "dy": 0.0, "dz": -0.02}
#     Home:      {"mode": "home"}
#     Fine:      {"mode": "fine"}
#   Per il modo "relative" il target assoluto viene calcolato sommando i
#   delta alla posizione ATTUALE di tool0 (letta via TF dal Nodo 2, metodo
#   pubblico posizione_attuale()).
#
#   NOTA TECNICA - import di moduli che iniziano con una cifra:
#   I file 06_verifica_raggiungibilita.py e 07_esegui_movimento_cartesiano.py
#   non sono importabili con una normale istruzione "import ..." perche' il
#   loro nome inizia con una cifra e non e' quindi un identificatore Python
#   valido. Si usa percio' importlib.import_module(), che accetta il nome del
#   modulo come stringa e non ha questo vincolo.
# ==============================================================================

import importlib
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import dei due nodi precedenti come moduli (nomi che iniziano con una cifra)
_mod_verifica = importlib.import_module('tutorial_nodi_base.06_verifica_raggiungibilita')
_mod_esecuzione = importlib.import_module('tutorial_nodi_base.07_esegui_movimento_cartesiano')

VerificaRaggiungibilita = _mod_verifica.VerificaRaggiungibilita
EsecuzioneMovimento = _mod_esecuzione.EsecuzioneMovimento

# Parola chiave che termina il ciclo da tastiera
PAROLA_FINE = 'fine'

# Nome del topic su cui arrivano i comandi vocali (JSON su String)
TOPIC_COMANDO_VOCALE = '/comando_vocale'


class InterfacciaContinua(Node):
    """
    Nodo di interfaccia: resta in ascolto (da tastiera o da topic vocale),
    verifica ed esegue i movimenti richiesti riutilizzando i nodi 1 e 2,
    finche' non riceve il comando di terminazione.
    """

    def __init__(self, nome_nodo='nodo_interfaccia_continua'):
        super().__init__(nome_nodo)

        # Parametro ROS per scegliere la modalita' di ascolto
        self.declare_parameter('modalita', 'tastiera')
        self._modalita = self.get_parameter('modalita').get_parameter_value().string_value

        self.get_logger().info(f'Nodo di interfaccia continua avviato (modalita\': {self._modalita}).')

        # I due nodi vengono creati come componenti: nel loro __init__ si
        # collegano gia' ai server/servizi MoveIt necessari.
        self.nodo_verifica = VerificaRaggiungibilita()
        self.nodo_esecuzione = EsecuzioneMovimento()

        # Sottoscrizione al topic vocale (attiva sempre, indipendentemente
        # dalla modalita': anche in modalita' tastiera un comando vocale
        # verrebbe comunque processato al prossimo giro di spin).
        self._sub_comando_vocale = self.create_subscription(
            String, TOPIC_COMANDO_VOCALE, self._callback_comando_vocale, 10
        )

        # Evita di eseguire due movimenti sovrapposti (es. comando vocale
        # arrivato mentre un altro movimento e' gia' in corso)
        self._occupato = False
        # Impostato a True quando arriva {"mode": "fine"} in modalita' vocale
        self._richiesta_fine = False

    # --------------------------------------------------------------------
    # Parsing di una riga "X Y Z" inserita da tastiera
    # --------------------------------------------------------------------
    @staticmethod
    def _analizza_coordinate(riga):
        """Converte una riga tipo '0.20 -0.10 0.35' (separatori: spazio o
        virgola) in tre float. Solleva ValueError se il formato non e'
        valido (gestito dal chiamante)."""
        pezzi = riga.replace(',', ' ').split()
        if len(pezzi) != 3:
            raise ValueError(
                f'Attese 3 coordinate (X Y Z), ricevute {len(pezzi)}.'
            )
        x, y, z = (float(p) for p in pezzi)
        return x, y, z

    # --------------------------------------------------------------------
    # Logica condivisa: verifica + esecuzione di un target assoluto (X,Y,Z)
    # Usata sia dal ciclo da tastiera, sia dal callback del topic vocale.
    # --------------------------------------------------------------------
    def _gestisci_movimento(self, x, y, z):
        # try/except ampio: una posizione "strana" puo' generare eccezioni
        # impreviste piu' in profondita' (es. traiettoria degenere). Senza
        # questa rete di sicurezza, un'eccezione qui risalirebbe fino al
        # ciclo principale e farebbe CHIUDERE l'intero nodo (bug osservato:
        # "il nodo si spegne se do posizioni strane"). Con il try/except il
        # nodo resta vivo e pronto a ricevere il comando successivo.
        try:
            esito_verifica = self.nodo_verifica.verifica_posa(x, y, z)
            print(esito_verifica['messaggio'])

            if not esito_verifica['raggiungibile']:
                print('↳ Movimento NON eseguito: posa non raggiungibile.\n')
                return

            esito_esecuzione = self.nodo_esecuzione.esegui_movimento_cartesiano(x, y, z)
            print(esito_esecuzione['messaggio'])
            print()

        except Exception as errore:
            self.get_logger().error(
                f'❌ Errore imprevisto durante il movimento verso '
                f'(X={x}, Y={y}, Z={z}): {errore!r}'
            )
            print(
                f'❌ Errore imprevisto, movimento annullato: {errore!r}\n'
                'Il nodo resta comunque attivo, prova con un\'altra posizione.\n'
            )

    def _gestisci_home(self):
        try:
            print('--- Rientro in HOME ---')
            esito_home = self.nodo_esecuzione.vai_a_home()
            print(esito_home['messaggio'])
            print()
        except Exception as errore:
            self.get_logger().error(f'❌ Errore imprevisto durante il rientro in HOME: {errore!r}')
            print(f'❌ Errore imprevisto durante il rientro in HOME: {errore!r}\n')

    # --------------------------------------------------------------------
    # Callback del topic vocale
    # --------------------------------------------------------------------
    def _callback_comando_vocale(self, msg):
        if self._occupato:
            self.get_logger().warn(
                'Comando vocale ignorato: un movimento e\' gia\' in corso.'
            )
            return

        self._occupato = True
        try:
            try:
                dati = json.loads(msg.data)
            except json.JSONDecodeError as errore:
                print(f'❌ Comando vocale non e\' un JSON valido: {errore} (payload: {msg.data!r})')
                return

            if not isinstance(dati, dict):
                print(f'❌ Comando vocale malformato (atteso un oggetto JSON, ricevuto: {dati!r})')
                return

            modo = dati.get('mode')

            if modo == 'fine':
                self.get_logger().info('Comando vocale "fine" ricevuto: termino il ciclo.')
                self._richiesta_fine = True

            elif modo == 'home':
                print('🎤 Comando vocale: HOME')
                self._gestisci_home()

            elif modo == 'absolute':
                try:
                    x, y, z = float(dati['x']), float(dati['y']), float(dati['z'])
                except (KeyError, TypeError, ValueError) as errore:
                    print(f'❌ Comando vocale "absolute" con campi mancanti/non validi: {errore}')
                    return
                print(f'🎤 Comando vocale ASSOLUTO: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')
                self._gestisci_movimento(x, y, z)

            elif modo == 'relative':
                try:
                    dx, dy, dz = float(dati['dx']), float(dati['dy']), float(dati['dz'])
                except (KeyError, TypeError, ValueError) as errore:
                    print(f'❌ Comando vocale "relative" con campi mancanti/non validi: {errore}')
                    return

                attuale = self.nodo_esecuzione.posizione_attuale()
                if attuale is None:
                    print('❌ Impossibile leggere la posizione attuale: comando ignorato.')
                    return

                x = attuale[0] + dx
                y = attuale[1] + dy
                z = attuale[2] + dz
                print(
                    f'🎤 Comando vocale RELATIVO: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f} '
                    f'-> target X={x:.3f}, Y={y:.3f}, Z={z:.3f}'
                )
                self._gestisci_movimento(x, y, z)

            else:
                print(f'❌ Comando vocale con "mode" non riconosciuto: {dati}')

        except Exception as errore:
            # Ultima rete di sicurezza: qualunque errore imprevisto nel
            # dispatch del comando viene solo segnalato, mai lasciato
            # propagare (altrimenti farebbe chiudere l'intero nodo).
            self.get_logger().error(f'❌ Errore imprevisto nel comando vocale: {errore!r}')
            print(f'❌ Errore imprevisto nel comando vocale: {errore!r}\n')

        finally:
            self._occupato = False

    # --------------------------------------------------------------------
    # Ciclo principale - modalita' TASTIERA
    # --------------------------------------------------------------------
    def _ciclo_tastiera(self):
        print('=== Interfaccia continua: controllo cartesiano da tastiera ===')
        print('Inserisci coordinate come "X Y Z" (in metri, rispetto a base_link).')
        print(f'Scrivi "{PAROLA_FINE}" per terminare e riportare il robot in HOME.\n')

        while True:
            try:
                riga = input('Coordinate (X Y Z) > ').strip()
            except EOFError:
                break

            if riga.lower() == PAROLA_FINE:
                self.get_logger().info(f'Parola chiave "{PAROLA_FINE}" ricevuta: termino il ciclo.')
                break

            if riga == '':
                continue

            try:
                x, y, z = self._analizza_coordinate(riga)
            except ValueError as errore:
                print(f'❌ Input non valido: {errore}')
                continue

            self._gestisci_movimento(x, y, z)

    # --------------------------------------------------------------------
    # Ciclo principale - modalita' VOCALE
    # --------------------------------------------------------------------
    def _ciclo_vocale(self):
        print('=== Interfaccia continua: in ascolto sul topic vocale ===')
        print(f'Topic: {TOPIC_COMANDO_VOCALE} (std_msgs/String, payload JSON).')
        print('In attesa di comandi dal nodo agente vocale (Ctrl+C per uscire).\n')

        while rclpy.ok() and not self._richiesta_fine:
            rclpy.spin_once(self, timeout_sec=0.2)

    # --------------------------------------------------------------------
    # Selezione della modalita'
    # --------------------------------------------------------------------
    def esegui_ciclo(self):
        if self._modalita == 'vocale':
            self._ciclo_vocale()
        else:
            self._ciclo_tastiera()

        # -- Fine ciclo (in entrambe le modalita'): rientro in sicurezza in HOME --
        self._gestisci_home()

    def distruggi(self):
        """Distrugge in ordine i nodi componenti e se stesso."""
        self.nodo_verifica.destroy_node()
        self.nodo_esecuzione.destroy_node()
        self.destroy_node()


# ==============================================================================
# Esecuzione del nodo
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    nodo = InterfacciaContinua()

    try:
        nodo.esegui_ciclo()
    except KeyboardInterrupt:
        print('\nInterrotto da tastiera (Ctrl+C): riporto comunque il robot in HOME...')
        try:
            nodo._gestisci_home()
        except Exception as errore:
            nodo.get_logger().error(f'Impossibile completare il rientro in HOME: {errore}')
    finally:
        nodo.distruggi()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
