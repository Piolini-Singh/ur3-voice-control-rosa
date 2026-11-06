#!/usr/bin/env python3
# ==============================================================================
# NODO 9 - AGENTE VOCALE (ROSA + Whisper + GPT-4o)
#
# Descrizione:
#   Nodo standalone, separato dai nodi 1-8 (tutorial_nodi_base): NON li importa
#   e NON li modifica. Si collega al sistema esistente unicamente tramite il
#   topic '/comando_vocale', gestito da agente_comune.py (tool ROSA condivisi
#   con 09_agente_vocale_testo.py, la versione da tastiera usata per i test).
#
#   Flusso:
#     1. L'operatore preme INVIO -> il nodo registra dal microfono.
#     2. L'operatore preme di nuovo INVIO -> la registrazione si ferma.
#     3. L'audio viene trascritto in italiano con OpenAI Whisper ('whisper-1').
#     4. Il testo trascritto viene passato a ROSA (GPT-4o + function-calling),
#        che chiama il tool giusto in agente_comune.py.
#
#   ESEGUIRE dalla radice del repository, con il virtualenv attivo:
#       source /opt/ros/humble/setup.bash
#       source venv_agente_vocale/bin/activate
#       python3 voice_agent/09_agente_vocale.py
# ==============================================================================

import os
import tempfile
import threading

import numpy as np
import sounddevice as sd
import soundfile as sf

from openai import OpenAI

import agente_comune

FREQUENZA_CAMPIONAMENTO = 16000  # Hz, compatibile con Whisper


def registra_audio(percorso_output):
    """Registra dal microfono da quando viene chiamata a quando l'utente
    preme di nuovo INVIO (push-to-talk)."""
    frames = []
    registrazione_attiva = threading.Event()
    registrazione_attiva.set()

    def callback(indata, frame_count, time_info, status):
        if registrazione_attiva.is_set():
            frames.append(indata.copy())

    stream = sd.InputStream(
        samplerate=FREQUENZA_CAMPIONAMENTO, channels=1, callback=callback
    )
    stream.start()
    input('🎙️  Registrazione in corso... premi di nuovo INVIO per fermare. ')
    registrazione_attiva.clear()
    stream.stop()
    stream.close()

    if not frames:
        return False

    audio = np.concatenate(frames, axis=0)
    sf.write(percorso_output, audio, FREQUENZA_CAMPIONAMENTO)
    return True


def trascrivi_audio(percorso_file, client_openai):
    with open(percorso_file, 'rb') as f:
        trascrizione = client_openai.audio.transcriptions.create(
            model='whisper-1',
            file=f,
            language='it',
        )
    return trascrizione.text


def main():
    agente = agente_comune.inizializza_agente()
    client_openai = OpenAI()

    print('=== Agente vocale UR3 (ROSA + Whisper + GPT-4o) ===')
    print(f'Pubblica su: {agente_comune.TOPIC_COMANDO_VOCALE}')
    print('Premi INVIO per iniziare a parlare, INVIO di nuovo per fermare.')
    print('Dì "fine" o "basta" per terminare (oppure Ctrl+C).\n')

    try:
        while not agente_comune.richiesta_fine:
            input('[Premi INVIO per registrare] ')

            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
                percorso_audio = tmp.name

            if not registra_audio(percorso_audio):
                print('❌ Nessun audio registrato.\n')
                continue

            print('Trascrizione in corso (Whisper)...')
            testo = trascrivi_audio(percorso_audio, client_openai)
            os.remove(percorso_audio)
            print(f'🗣️  Hai detto: "{testo}"')

            print('Interpretazione in corso (GPT-4o via ROSA)...')
            risposta = agente.invoke(testo)
            print(f'🤖 {risposta}\n')

    except KeyboardInterrupt:
        print('\nInterrotto da tastiera (Ctrl+C).')
    finally:
        agente_comune.chiudi()


if __name__ == '__main__':
    main()
