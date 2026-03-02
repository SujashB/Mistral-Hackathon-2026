# MotionCorps — Unitree G1 Vision-Language-Action Robot

> Fine-tuned Ministral-8B · LLaVA Vision Grounding · ROS 2 · ElevenLabs Voice

MotionCorps is a fully embodied humanoid robot control system built for the Mistral Hackathon 2026. A fine-tuned Ministral-8B model acts as the central policy — receiving natural language commands, understanding the visual scene via LLaVA, and emitting precise joint angles at 60 Hz to control a Unitree G1 (29 DOF) robot visualised in RViz. ElevenLabs handles both voice input (STT) and voice output (TTS), closing the full spoken-language loop.

---

## Short Description

You speak a command. The robot sees the scene. Ministral decides what to do. The robot moves and speaks back.

```
Microphone → ElevenLabs STT → /voice_command
    → LLaVA (scene grounding) → /world_state
    → Ministral VLA (fine-tuned) → joint_positions JSON
        → 60 Hz interpolation → /joint_states → RViz
    → ElevenLabs TTS (Rachel) → speaker
```

---

## Architecture

![MotionCorps system design](./SystemDesignMinistral.png)

### ROS 2 Nodes

| Node | Role |
|---|---|
| `mistral_joint_commander` | LLM inference + 60 Hz joint publisher |
| `vision_grounder` | LLaVA scene understanding → `/world_state` |
| `behavior_manager` | State machine + ElevenLabs TTS relay |
| `energy_monitor` | Balance scoring → `/energy_state` |
| `scene_marker_publisher` | RViz 3D object markers from world state |

### ROS 2 Topic Map

| Topic | Type | Flow |
|---|---|---|
| `/voice_command` | String | CLI/STT → commander, behavior_manager, vision_grounder |
| `/world_state` | String | vision_grounder → commander, behavior_manager |
| `/joint_states` | JointState | commander → robot_state_publisher, behavior_manager |
| `/robot_state` | String | behavior_manager → commander |
| `/ministral_speech` | String | commander → behavior_manager (TTS) |
| `/vision_request` | String | behavior_manager, commander → vision_grounder |
| `/energy_state` | String | energy_monitor → behavior_manager |
| `/scene_markers` | MarkerArray | scene_marker_publisher → RViz |

---

## How It Was Built

### Step 1 — Dataset Generation

`training/generate_dataset.py` synthesises a motion-command dataset for all 29 revolute joints of the Unitree G1. Every sample is a chat turn:

```json
{
  "messages": [
    {"role": "system",    "content": "You are a robot motion controller for a Unitree G1 humanoid robot with 29 DOF. Given a natural language motion command, output target joint positions as JSON. All values in radians. Output ONLY the JSON object with a 'joint_positions' key."},
    {"role": "user",      "content": "Raise both arms above your head"},
    {"role": "assistant", "content": "{\"joint_positions\":{\"left_shoulder_pitch_joint\":-2.5,...}}"}
  ]
}
```

The dataset covers:
- **20 static named poses** — stand, arms_up, t_pose, wave, bow, squat, lunge, crouch, etc.
- **10 parameterised poses** — raise arm by N degrees, bend knees N %, turn waist, lean forward
- **13 composite poses** — combinations of arm, leg, and waist movements

Each pose was augmented with **4–8 natural language phrasings**, producing:

| Split | Samples |
|---|---|
| Train | 3,827 |
| Validation | 476 |
| Test | 485 |
| **Total** | **4,788** |

All 29 joint values are validated against URDF hard limits before writing to disk.

---

### Step 2 — Fine-Tuning with Hugging Face TRL (QLoRA)

**Base model:** `mistralai/Ministral-8B-Instruct-2410`

We used QLoRA — 4-bit quantised base weights with trainable LoRA adapters — via Hugging Face `trl` (`SFTTrainer`) and `peft`:

```python
# 4-bit quantisation
BitsAndBytesConfig(load_in_4bit=True, bnb_4bit_quant_type="nf4",
                   bnb_4bit_compute_dtype=bfloat16, bnb_4bit_use_double_quant=True)

# LoRA adapters injected into all projection layers
LoraConfig(r=32, lora_alpha=64, lora_dropout=0.05,
           target_modules=["q_proj","k_proj","v_proj","o_proj",
                           "gate_proj","up_proj","down_proj"])
```

**Training hyperparameters:**

| Parameter | Value |
|---|---|
| Base model | Ministral-8B-Instruct-2410 |
| Epochs | 5 |
| Effective batch size | 16 (batch=1 × grad_accum=16) |
| Learning rate | 2e-4 |
| LR scheduler | Cosine |
| Warmup ratio | 0.05 |
| Weight decay | 0.01 |
| Optimiser | paged_adamw_8bit |
| Max sequence length | 768 tokens |
| Precision | bf16 |

LoRA adapters are saved to `training/output/final/`. The merged model is exported to GGUF for Ollama: `training/output/g1-controller.gguf`.

---

### Step 3 — Evaluation Results

Evaluated on 485 held-out test samples:

```
============================================================
EVALUATION RESULTS
============================================================
Total test samples:       485
JSON validity rate:       100.0%     [target: >99%]   ✓
Joint limit compliance:   100.0%     [target: >99%]   ✓
Mean Absolute Error:      0.0003 rad [target: <0.15]  ✓
Pose accuracy (0.1 tol):  98.4%      [target: >90%]   ✓
============================================================
```

The fine-tuned model always produces valid, in-bounds JSON and matches ground-truth poses within 0.1 rad on 98.4 % of test inputs.

---

### Step 4 — LLaVA Visual Grounding

`vision_grounder.py` runs `llava:7b` via Ollama on the robot's forward camera. It outputs structured JSON:

```json
{
  "scene_description": "A wooden table with a red bottle on the left side.",
  "robot_context": {
    "table_distance_m": 0.95,
    "reachable_objects": ["red bottle"],
    "recommended_target": "red bottle"
  },
  "objects": [
    {"name": "red bottle", "spatial_relation_to_robot": "ahead-left",
     "distance_m": 0.8, "confidence": 0.95}
  ]
}
```

This world state is injected into every Ministral prompt so the model can resolve references like *"point at it"* to a specific object and direction.

Vision inference fires automatically when voice commands mention scene keywords (`table`, `bottle`, `point`, `what`, `see`, etc.).

---

### Step 5 — ROS 2 Integration

**`mistral_joint_commander`** uses a two-thread design:
- **Thread A (LLM)** — receives the voice command, injects world state, calls Ministral via Ollama (`g1-controller`), parses `joint_positions` JSON
- **Thread B (60 Hz timer)** — smoothly interpolates current positions toward target and publishes `/joint_states`

**`behavior_manager`** is a state machine:
```
IDLE → PERCEIVING → PLANNING → EXECUTING → IDLE
                                    ↕
                              STABILIZING
```
It monitors joint motion (threshold: 0.08 rad), enforces a 35-second execution timeout, and routes speech to ElevenLabs.

---

### Step 6 — ElevenLabs Voice Loop

**Speech-to-Text (voice input):**
`voice_input.py` records microphone audio with `sounddevice`, wraps it as WAV, and sends it to `https://api.elevenlabs.io/v1/speech-to-text` (model: `scribe_v1`). The transcribed command is published to `/voice_command`.

**Text-to-Speech (voice output):**
`behavior_manager` calls `https://api.elevenlabs.io/v1/text-to-speech/{voice_id}` (voice: Rachel). A short action description is generated by `mistral_joint_commander` from the command and current world state — and spoken *before* the LLM inference completes, so the robot's voice is synchronised with the start of motion.

---

## Quickstart

### Prerequisites

- ROS 2 Jazzy or Humble
- Python 3.10+
- [Ollama](https://ollama.com) running locally
- NVIDIA GPU recommended (for local PEFT) or CPU (Ollama GGUF)

### 1. Clone and build

```bash
git clone https://github.com/your-org/Mistral-Hackathon-2026
cd Mistral-Hackathon-2026
cd ws_g1 && colcon build --packages-select g1_package && cd ..
```

### 2. Configure credentials

```bash
echo "ELEVENLABS_API_KEY=your_key_here" > .env
```

### 3. Pull vision model

```bash
ollama pull llava:7b
```

### 4. Launch

```bash
# Text input
./launch_g1_mistral.sh

# Voice input/output via ElevenLabs
./launch_g1_mistral.sh --voice
```

### 5. Example commands

```
motioncorps> raise your right arm
motioncorps> point at the bottle
motioncorps> do a t pose
motioncorps> put your left leg forward
motioncorps> what is on the table
```

---

## Training from Scratch

```bash
pip install -r training/requirements.txt

python training/generate_dataset.py   # ~4,800 JSONL samples
python training/train.py              # QLoRA fine-tuning (~2–4 h on A100)
python training/evaluate.py           # metrics on test split
```

---

## Project Structure

```
Mistral-Hackathon-2026/
├── training/
│   ├── generate_dataset.py     # Pose dataset synthesis (4,788 samples)
│   ├── train.py                # QLoRA fine-tuning via trl SFTTrainer
│   ├── evaluate.py             # JSON validity, MAE, pose accuracy
│   ├── model_config.py         # Base model path + HF auth
│   ├── requirements.txt
│   └── output/
│       ├── final/              # LoRA adapters
│       ├── g1-controller.gguf  # Ollama-ready export
│       └── Modelfile           # Ollama model definition
├── ws_g1/
│   └── g1_package/
│       ├── scripts/            # ROS 2 Python nodes
│       ├── launch/             # g1_mistral.launch.xml
│       ├── config/             # vla_params.yaml
│       └── urdf/               # G1 29-DOF URDF + RViz config
├── voice_input.py              # ElevenLabs STT push-to-talk
├── launch_g1_mistral.sh        # One-command launcher + CLI
└── .env                        # API keys (not committed)
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| Base LLM | Ministral-8B-Instruct-2410 (Mistral AI) |
| Fine-tuning | Hugging Face TRL · PEFT (QLoRA) · bitsandbytes |
| Vision grounding | LLaVA 7B via Ollama |
| LLM serving | Ollama (GGUF) / local PEFT adapters |
| Robot framework | ROS 2 Jazzy / Humble |
| Visualisation | RViz 2 |
| Voice I/O | ElevenLabs STT (`scribe_v1`) + TTS (Rachel) |
| Audio playback | sounddevice · soundfile |
| Robot model | Unitree G1 — 29 DOF URDF (RViz only, no physics) |
