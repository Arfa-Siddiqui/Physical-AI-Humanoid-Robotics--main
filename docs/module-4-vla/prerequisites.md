---
sidebar_position: 0
---

# Prerequisites: AI Models for Embodied Intelligence

Vision-Language-Action (VLA) systems combine perception, language understanding, and motor control. This chapter prepares your environment for integrating large language models (LLMs) and vision transformers into robotic systems.

## Why VLA?

Traditional robots require task-specific programming for every action. VLA systems enable:

- **Natural language commands**: "Bring me the red cup from the kitchen"
- **Generalization**: Transfer knowledge from internet-scale training to novel objects
- **Multi-modal understanding**: Combine vision, language, and proprioception

**Example:** RT-2 (Robotics Transformer 2) learns from web images and can pick objects it never saw during robot training (Brohan et al., 2023).

## System Requirements

**Minimum:**
- **GPU**: 8 GB VRAM (RTX 3060 Ti) for small models (7B parameters)
- **RAM**: 16 GB
- **Storage**: 50 GB for model weights

**Recommended:**
- **GPU**: 24 GB VRAM (RTX 3090/4090) for medium models (13B-70B)
- **RAM**: 32 GB
- **Storage**: NVMe SSD for fast model loading

**Cloud Alternative:** Use OpenAI API (GPT-4) or Anthropic Claude for serverless LLM access.

## Installing Python Dependencies

```bash
pip3 install --upgrade pip
pip3 install transformers torch torchvision torchaudio accelerate
pip3 install openai anthropic  # For API-based models
pip3 install whisper-timestamped  # For speech recognition
pip3 install langchain  # For LLM orchestration
```

**Verify PyTorch GPU Support:**

```python
import torch
print(torch.cuda.is_available())  # Should print: True
print(torch.cuda.get_device_name(0))  # e.g., "NVIDIA GeForce RTX 3070"
```

## Speech Recognition: Whisper

**Whisper** (OpenAI) provides state-of-the-art speech-to-text.

### Install Whisper

```bash
pip3 install openai-whisper
```

### Test Whisper

```python
import whisper

model = whisper.load_model("base")  # Options: tiny, base, small, medium, large
result = model.transcribe("audio.mp3")
print(result["text"])
```

**Models:**
- **tiny**: 39M params, 32× faster, moderate accuracy
- **base**: 74M params, good balance
- **small**: 244M params, high accuracy
- **large**: 1.5B params, best accuracy (requires 10 GB VRAM)

## Language Models: HuggingFace Transformers

### Local LLM (LLaMA 2)

Download and run LLaMA 2 locally:

```bash
# Install llama-cpp-python for efficient inference
pip3 install llama-cpp-python

# Download model (example: 7B quantized)
wget https://huggingface.co/TheBloke/Llama-2-7B-GGUF/resolve/main/llama-2-7b.Q4_K_M.gguf
```

**Inference:**

```python
from llama_cpp import Llama

llm = Llama(model_path="llama-2-7b.Q4_K_M.gguf", n_ctx=2048)
output = llm("Q: How do I pick up a cup? A:", max_tokens=100)
print(output['choices'][0]['text'])
```

### Cloud LLM (OpenAI GPT-4)

**Set API Key:**

```bash
export OPENAI_API_KEY="sk-..."
```

**Usage:**

```python
import openai

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "You are a robotic assistant."},
        {"role": "user", "content": "How do I navigate to the kitchen?"}
    ]
)
print(response.choices[0].message.content)
```

**Cost:** GPT-4: ~$0.03/1K tokens. Budget accordingly for experimentation.

## Vision Models: CLIP and OWL-ViT

### CLIP (Contrastive Language-Image Pretraining)

CLIP aligns images and text in a shared embedding space.

```bash
pip3 install transformers pillow
```

**Example: Zero-Shot Image Classification**

```python
from transformers import CLIPProcessor, CLIPModel
from PIL import Image

model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

image = Image.open("robot_view.jpg")
inputs = processor(
    text=["a red cup", "a blue bottle", "a green apple"],
    images=image,
    return_tensors="pt",
    padding=True
)

outputs = model(**inputs)
logits_per_image = outputs.logits_per_image
probs = logits_per_image.softmax(dim=1)
print(f"Probabilities: {probs}")  # [0.7, 0.2, 0.1] → likely "red cup"
```

### OWL-ViT (Open-World Localization)

Detect objects via text prompts:

```python
from transformers import OwlViTProcessor, OwlViTForObjectDetection

processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")

image = Image.open("scene.jpg")
texts = [["a red cup", "a person"]]

inputs = processor(text=texts, images=image, return_tensors="pt")
outputs = model(**inputs)

# Post-process to get bounding boxes
target_sizes = torch.Tensor([image.size[::-1]])
results = processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.1)

for box, score, label in zip(results[0]["boxes"], results[0]["scores"], results[0]["labels"]):
    print(f"Detected '{texts[0][label]}' at {box} with confidence {score:.2f}")
```

## ROS 2 Integration Libraries

### LangChain for ROS

```bash
pip3 install langchain langchain-community
```

**Example: LLM-Based Planner**

```python
from langchain.llms import OpenAI
from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate

llm = OpenAI(temperature=0.7)
template = "You are a robot. User says: '{command}'. Decompose into steps:"
prompt = PromptTemplate(input_variables=["command"], template=template)
chain = LLMChain(llm=llm, prompt=prompt)

result = chain.run(command="Bring me the red cup from the kitchen")
print(result)
# Output: "1. Navigate to kitchen. 2. Locate red cup. 3. Grasp cup. 4. Return to user."
```

## Testing the Stack

### Voice → Text → Action Pipeline

**Record Audio:**

```bash
arecord -d 5 -f cd test.wav  # Record 5 seconds
```

**Transcribe:**

```python
import whisper
model = whisper.load_model("base")
result = model.transcribe("test.wav")
print(f"You said: {result['text']}")
```

**Plan with LLM:**

```python
import openai
openai.api_key = "sk-..."

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[{"role": "user", "content": f"Robot task: {result['text']}. List steps."}]
)
print(response.choices[0].message.content)
```

**Execute (Placeholder):**

```bash
# Later chapters will integrate with ROS 2 action servers
ros2 action send_goal /navigate_to_pose ...
```

## GPU Memory Management

Large models consume significant VRAM. Monitor usage:

```bash
watch -n 1 nvidia-smi
```

**Optimization:**
- Use **quantized models** (4-bit, 8-bit) for 50% memory reduction
- **Offload to CPU** for inference (slower but feasible on CPU-only systems)

## Next Steps

With speech recognition, language models, and vision transformers installed, you're ready to build VLA systems. The next chapter integrates Whisper for voice commands and creates a voice-to-action pipeline.

## References

- Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv preprint arXiv:2307.15818*.
- Radford, A., et al. (2021). "Learning Transferable Visual Models From Natural Language Supervision (CLIP)." *ICML 2021*.
- Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision (Whisper)." OpenAI.
