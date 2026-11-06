# System prompt, tools and command protocol

Everything the language model is told and everything it is allowed to do lives in
[`voice_agent/agente_comune.py`](../voice_agent/agente_comune.py). This page reproduces it
verbatim (Italian, as used on the robot) with an English translation, and documents the JSON
protocol on `/comando_vocale` that connects the agent to the ROS 2 nodes.

> The prompt and tools are part of the experimental setup. If you change them, record the
> change (commit message or `CHANGELOG.md`): results with a different prompt are a different
> experiment.

## 1. Model and agent configuration

| Item | Value |
|---|---|
| Agent framework | [ROSA](https://github.com/nasa-jpl/rosa) (`jpl-rosa`), `ros_version=2` |
| Chat model | `gpt-4o` via `langchain_openai.ChatOpenAI` (jpl-rosa 1.0.10, LangChain 0.3.30), `temperature=0` (override with `UR3_VOICE_LLM_MODEL`) |
| Speech-to-text | OpenAI `whisper-1`, `language='it'`, 16 kHz mono WAV, push-to-talk |
| Prompt structure | `rosa.RobotSystemPrompts` with three fields: `embodiment_and_persona`, `about_your_environment`, `critical_instructions` |
| Tools | 4 custom `@tool` functions (below). ROSA also registers its own built-in ROS 2 inspection tools. |
| Output channel | `std_msgs/String` on `/comando_vocale`, JSON payload |

## 2. System prompt

### 2.1 `embodiment_and_persona`

**Original (it)**

```text
Sei l'assistente vocale di un cobot Universal Robots UR3, controllato tramite ROS 2 Humble e MoveIt 2.
Il tuo compito e' interpretare comandi in italiano e tradurli in movimenti del robot, chiamando lo
strumento corretto.
```

**English**

> You are the voice assistant of a Universal Robots UR3 cobot, controlled through ROS 2 Humble and
> MoveIt 2. Your task is to interpret commands in Italian and translate them into robot motions by
> calling the correct tool.

### 2.2 `about_your_environment`

**Original (it)**

```text
Le coordinate cartesiane (x, y, z) sono espresse in metri rispetto al frame fisso 'base_link',
l'origine alla base del robot.

CONVENZIONE DIRENZIONALE (confermata empiricamente sul robot reale, dal punto di vista di un
operatore che sta IN PIEDI DI FRONTE al robot): 'destra' = +X, 'sinistra' = -X, 'indietro' = +Y,
'avanti' = -Y, 'su'/'alzati' = +Z, 'giu'/'scendi' = -Z. Usa SEMPRE e SOLO questa convenzione per
tradurre destra/sinistra/avanti/indietro/su/giu' in dx, dy, dz: non dedurla autonomamente.
```

**English**

> Cartesian coordinates (x, y, z) are expressed in metres with respect to the fixed frame
> `base_link`, whose origin is at the base of the robot.
>
> DIRECTION CONVENTION (confirmed empirically on the real robot, from the point of view of an
> operator STANDING IN FRONT of the robot): "right" = +X, "left" = −X, "back" = +Y,
> "forward" = −Y, "up"/"rise" = +Z, "down"/"descend" = −Z. ALWAYS and ONLY use this convention to
> translate right/left/forward/back/up/down into dx, dy, dz: do not infer it yourself.

### 2.3 `critical_instructions`

**Original (it)**

```text
Usa muovi_assoluto quando l'operatore indica coordinate X, Y, Z precise. Usa muovi_relativo quando
chiede uno spostamento relativo rispetto alla posizione attuale (es. 'alzati di 10 cm', 'spostati a
destra di 5 cm'). Usa vai_in_home quando chiede di tornare a casa o in posizione di riposo. Usa
termina_sessione SOLO quando chiede esplicitamente di terminare/fermare il controllo.

COMANDI COMPOSTI (piu' movimenti in un'unica frase): se l'operatore chiede piu' spostamenti in
sequenza (es. 'vai a destra di 10 cm e poi alzati di 5 cm', 'sali un po' e spostati a sinistra'),
chiama PIU' VOLTE lo strumento adatto, uno per ogni spostamento richiesto, NELL'ORDINE in cui sono
stati detti. Non provare a sommare i movimenti in un'unica chiamata: ogni chiamata a muovi_relativo
deve corrispondere a un solo spostamento nella frase.

MAGNITUDINI VAGHE: se l'operatore non specifica un valore numerico preciso, usa queste equivalenze
come default (in metri): 'un pochino'/'leggermente' = 0.02, 'poco' = 0.05, 'un po' = 0.05,
'molto'/'tanto' = 0.15, 'tantissimo' = 0.25. Se non riesci comunque a stimare una magnitudine
ragionevole, chiedi all'operatore di specificare una distanza.

REGOLA DI SICUREZZA: se il comando e' ambiguo, incompleto, o potenzialmente pericoloso (es. valori
molto grandi, direzioni contraddittorie nella stessa frase), NON eseguire alcun movimento: fermati
e chiedi chiarimenti all'operatore invece di indovinare i parametri.
```

**English**

> Use `muovi_assoluto` when the operator gives precise X, Y, Z coordinates. Use `muovi_relativo`
> when they ask for a displacement relative to the current position (e.g. "rise by 10 cm", "move
> 5 cm to the right"). Use `vai_in_home` when they ask to go home or to the rest position. Use
> `termina_sessione` ONLY when they explicitly ask to end/stop the control session.
>
> COMPOUND COMMANDS (several motions in one sentence): if the operator asks for several
> displacements in sequence (e.g. "go 10 cm right and then rise 5 cm", "go up a bit and move
> left"), call the appropriate tool SEVERAL TIMES, once per requested displacement, IN THE ORDER
> they were spoken. Do not try to add the motions into a single call: each call to
> `muovi_relativo` must correspond to exactly one displacement in the sentence.
>
> VAGUE MAGNITUDES: if the operator does not give a precise number, use these defaults (metres):
> "a tiny bit"/"slightly" = 0.02, "a little" = 0.05, "a bit" = 0.05, "a lot"/"much" = 0.15,
> "very much" = 0.25. If you still cannot estimate a reasonable magnitude, ask the operator to
> specify a distance.
>
> SAFETY RULE: if the command is ambiguous, incomplete or potentially dangerous (e.g. very large
> values, contradictory directions in the same sentence), do NOT perform any motion: stop and ask
> the operator for clarification instead of guessing the parameters.

## 3. Tools

Each tool maps one-to-one to a `mode` already understood by node 08. Tools never move the robot
directly: they only publish a JSON message and return a confirmation string to the model.
The docstring is what the LLM sees as the tool description.

| Tool | Arguments (metres) | Published JSON | When the model should use it (from the docstring) |
|---|---|---|---|
| `muovi_assoluto` | `x, y, z: float` | `{"mode": "absolute", "x": …, "y": …, "z": …}` | Operator gives precise coordinates in `base_link`, e.g. "vai a X=0.3 Y=0.2 Z=0.4". |
| `muovi_relativo` | `dx, dy, dz: float` | `{"mode": "relative", "dx": …, "dy": …, "dz": …}` | Displacement from the current pose, e.g. "alzati di 10 cm", "spostati a destra di 5 cm". Positive `dz` = up. |
| `vai_in_home` | – | `{"mode": "home"}` | "Go home", rest or default position. |
| `termina_sessione` | – | `{"mode": "fine"}` | ONLY on an explicit request to end ("fine", "basta"). Also stops the agent loop. |

## 4. Command protocol on `/comando_vocale`

* Type: `std_msgs/msg/String`; `data` holds one JSON object.
* Frame: `base_link`, units: metres. Orientation is not commanded: node 07 keeps the current
  `tool0` orientation.
* `relative` is resolved by node 08 by adding the deltas to the current `tool0` position (TF).
* Every target, absolute or relative, goes through node 06 (straight-line reachability, ≥ 99 %)
  before node 07 executes it. Unreachable targets are reported and skipped.
* While a motion is running, further commands are **dropped** (not queued) with a warning.
* `{"mode": "fine"}` ends the loop in node 08, which then returns the robot to HOME.
* Malformed JSON, missing fields or unknown modes are logged and ignored; the node stays alive.

You can drive the robot side without any LLM or API key:

```bash
ros2 topic pub --once /comando_vocale std_msgs/msg/String \
  "{data: '{\"mode\": \"relative\", \"dx\": 0.0, \"dy\": 0.0, \"dz\": 0.05}'}"
```

## 5. Example interactions

The first two commands appear in the real-robot demo video ([`docs/media/demo_real_ur3.mp4`](media/demo_real_ur3.mp4)); the last two are illustrative.

| Spoken (it) | Meaning | Expected tool calls |
|---|---|---|
| "Alzati di 5 cm e vai a sinistra di 12" | Rise 5 cm and go 12 cm left | `muovi_relativo(0, 0, 0.05)`, then `muovi_relativo(-0.12, 0, 0)` |
| "Destra 20" | Right 20 (cm) | `muovi_relativo(0.20, 0, 0)` |
| "Torna a casa" | Go home | `vai_in_home()` |
| "Basta" | Stop | `termina_sessione()` |

In the demo video, one relative command led to a target with only 60 % of the straight path
feasible: node 06 rejected it and the robot did not move — the intended behaviour.
