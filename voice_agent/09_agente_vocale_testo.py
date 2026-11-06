#!/usr/bin/env python3
# ==============================================================================
# NODO 9 (VERSIONE DI TEST) - AGENTE DA TASTIERA (ROSA + GPT-4o, senza audio)
#
# Descrizione:
#   Versione semplificata di 09_agente_vocale.py per testare la pipeline
#   ROSA + GPT-4o + pubblicazione su /comando_vocale SENZA la complessita'
#   di microfono/Whisper: qui l'operatore scrive direttamente il comando in
#   italiano nel terminale, come se fosse gia' stato trascritto.
#
#   Usa esattamente gli stessi tool e prompt di 09_agente_vocale.py (importati
#   da agente_comune.py): se un comando funziona qui, la logica di
#   interpretazione/pubblicazione e' la stessa che funzionera' con l'audio -
#   l'unica differenza e' la sorgente del testo (tastiera vs Whisper).
#
#   ESEGUIRE dalla radice del repository, con il virtualenv attivo:
#       source /opt/ros/humble/setup.bash
#       source venv_agente_vocale/bin/activate
#       python3 voice_agent/09_agente_vocale_testo.py
# ==============================================================================

import agente_comune


def main():
    agente = agente_comune.inizializza_agente()

    print('=== Agente da tastiera UR3 (ROSA + GPT-4o, senza audio) ===')
    print(f'Pubblica su: {agente_comune.TOPIC_COMANDO_VOCALE}')
    print('Scrivi un comando in italiano (es. "alzati di 10 cm", "vai a casa").')
    print('Scrivi "fine" o "basta" per terminare (oppure Ctrl+C).\n')

    try:
        while not agente_comune.richiesta_fine:
            testo = input('Comando > ').strip()
            if not testo:
                continue

            print('Interpretazione in corso (GPT-4o via ROSA)...')
            risposta = agente.invoke(testo)
            print(f'🤖 {risposta}\n')

    except KeyboardInterrupt:
        print('\nInterrotto da tastiera (Ctrl+C).')
    finally:
        agente_comune.chiudi()


if __name__ == '__main__':
    main()
