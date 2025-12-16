import os
import json
from typing import AsyncGenerator
import requests
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
env_path = Path(__file__).parent.parent.parent.parent / "src" / ".env"
load_dotenv(env_path)

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
GEMINI_MODEL = "gemini-2.5-flash"  # Available stable model (verified from API)


class GeminiService:
    def __init__(self):
        self.api_key = GEMINI_API_KEY
        self.model = GEMINI_MODEL
        self.base_url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}"

    async def generate_stream(
        self,
        prompt: str,
        system_instruction: str
    ) -> AsyncGenerator[str, None]:
        """
        Generate streaming response from Gemini.

        Args:
            prompt: User prompt
            system_instruction: System instruction for grounding

        Yields:
            Chunks of generated text
        """
        url = f"{self.base_url}:streamGenerateContent?alt=sse&key={self.api_key}"

        payload = {
            "contents": [
                {
                    "parts": [{"text": prompt}]
                }
            ],
            "systemInstruction": {
                "parts": [{"text": system_instruction}]
            },
            "generationConfig": {
                "temperature": 0.1,  # Lower for faster, more focused responses
                "topP": 0.95,
                "topK": 20,  # Reduced from 40 for faster generation
                "maxOutputTokens": 1024,  # Reduced from 2048 for faster responses
            }
        }

        headers = {
            "Content-Type": "application/json"
        }

        try:
            response = requests.post(
                url,
                headers=headers,
                json=payload,
                stream=True,
                timeout=30  # Reduced from 60 to 30 seconds
            )
            response.raise_for_status()

            for line in response.iter_lines():
                if line:
                    line_str = line.decode('utf-8')
                    if line_str.startswith('data: '):
                        json_str = line_str[6:]
                        try:
                            data = json.loads(json_str)
                            if 'candidates' in data and len(data['candidates']) > 0:
                                candidate = data['candidates'][0]
                                if 'content' in candidate and 'parts' in candidate['content']:
                                    parts = candidate['content']['parts']
                                    for part in parts:
                                        if 'text' in part:
                                            yield part['text']
                        except json.JSONDecodeError:
                            continue

        except requests.exceptions.RequestException as e:
            yield f"Error: {str(e)}"

    def generate_sync(
        self,
        prompt: str,
        system_instruction: str
    ) -> str:
        """
        Generate non-streaming response from Gemini.

        Args:
            prompt: User prompt
            system_instruction: System instruction for grounding

        Returns:
            Complete generated text
        """
        url = f"{self.base_url}:generateContent?key={self.api_key}"

        payload = {
            "contents": [
                {
                    "parts": [{"text": prompt}]
                }
            ],
            "systemInstruction": {
                "parts": [{"text": system_instruction}]
            },
            "generationConfig": {
                "temperature": 0.1,  # Lower for faster, more focused responses
                "topP": 0.95,
                "topK": 20,  # Reduced from 40 for faster generation
                "maxOutputTokens": 1024,  # Reduced from 2048 for faster responses
            }
        }

        headers = {
            "Content-Type": "application/json"
        }

        try:
            response = requests.post(
                url,
                headers=headers,
                json=payload,
                timeout=20  # Reduced for faster response
            )
            response.raise_for_status()

            result = response.json()
            if 'candidates' in result and len(result['candidates']) > 0:
                candidate = result['candidates'][0]
                if 'content' in candidate and 'parts' in candidate['content']:
                    parts = candidate['content']['parts']
                    text = ''.join([part.get('text', '') for part in parts])
                    return text

            return "No response generated."

        except requests.exceptions.RequestException as e:
            return f"Error: {str(e)}"


gemini_service = GeminiService()
