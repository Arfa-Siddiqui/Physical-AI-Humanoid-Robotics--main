#!/usr/bin/env python3
"""
Test script for RAG API endpoints.
Run after starting the backend server.
"""

import requests
import json
import sys

# Fix Windows console encoding
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding='utf-8')


API_BASE = "http://localhost:8000"


def test_health():
    """Test health endpoint."""
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{API_BASE}/health")
        response.raise_for_status()
        print(f"✓ Health check passed: {response.json()}")
        return True
    except Exception as e:
        print(f"✗ Health check failed: {e}")
        return False


def test_chat_health():
    """Test chat health endpoint."""
    print("\nTesting chat health endpoint...")
    try:
        response = requests.get(f"{API_BASE}/chat/health")
        response.raise_for_status()
        print(f"✓ Chat health check passed: {response.json()}")
        return True
    except Exception as e:
        print(f"✗ Chat health check failed: {e}")
        return False


def test_full_book_query():
    """Test full book RAG query."""
    print("\nTesting full book RAG query...")
    try:
        response = requests.post(
            f"{API_BASE}/chat/query",
            json={"question": "What is ROS2?"},
            stream=True,
            timeout=30
        )
        response.raise_for_status()

        print("✓ Streaming response:")
        chunks = []
        for line in response.iter_lines():
            if line:
                line_str = line.decode('utf-8')
                if line_str.startswith('data: '):
                    data_str = line_str[6:]
                    try:
                        data = json.loads(data_str)
                        if data.get('content'):
                            chunks.append(data['content'])
                            print(data['content'], end='', flush=True)
                        if data.get('done'):
                            break
                    except json.JSONDecodeError:
                        continue

        print("\n✓ Full book query completed")
        return len(chunks) > 0

    except Exception as e:
        print(f"\n✗ Full book query failed: {e}")
        return False


def test_selected_text_query():
    """Test selected text only query."""
    print("\nTesting selected text query...")
    try:
        response = requests.post(
            f"{API_BASE}/chat/selected-text",
            json={
                "question": "What does ROS stand for?",
                "selected_text": "ROS (Robot Operating System) is a flexible framework for writing robot software."
            },
            stream=True,
            timeout=30
        )
        response.raise_for_status()

        print("✓ Streaming response:")
        chunks = []
        for line in response.iter_lines():
            if line:
                line_str = line.decode('utf-8')
                if line_str.startswith('data: '):
                    data_str = line_str[6:]
                    try:
                        data = json.loads(data_str)
                        if data.get('content'):
                            chunks.append(data['content'])
                            print(data['content'], end='', flush=True)
                        if data.get('done'):
                            break
                    except json.JSONDecodeError:
                        continue

        print("\n✓ Selected text query completed")
        return len(chunks) > 0

    except Exception as e:
        print(f"\n✗ Selected text query failed: {e}")
        return False


def test_sources():
    """Test sources endpoint."""
    print("\nTesting sources endpoint...")
    try:
        response = requests.post(
            f"{API_BASE}/chat/sources",
            json={"question": "What is URDF?"}
        )
        response.raise_for_status()
        result = response.json()

        print(f"✓ Retrieved {len(result['sources'])} sources:")
        for idx, source in enumerate(result['sources'], 1):
            print(f"  {idx}. {source['file_path']} (score: {source['score']:.3f})")

        return len(result['sources']) > 0

    except Exception as e:
        print(f"✗ Sources query failed: {e}")
        return False


def test_error_handling():
    """Test error handling."""
    print("\nTesting error handling...")
    try:
        # Test empty question
        response = requests.post(
            f"{API_BASE}/chat/query",
            json={"question": ""}
        )
        if response.status_code == 400:
            print("✓ Empty question validation working")
        else:
            print("✗ Empty question should return 400")
            return False

        # Test missing selected text
        response = requests.post(
            f"{API_BASE}/chat/selected-text",
            json={"question": "test", "selected_text": ""}
        )
        if response.status_code == 400:
            print("✓ Empty selected text validation working")
        else:
            print("✗ Empty selected text should return 400")
            return False

        return True

    except Exception as e:
        print(f"✗ Error handling test failed: {e}")
        return False


def run_all_tests():
    """Run all tests."""
    print("="*60)
    print("RAG API Test Suite")
    print("="*60)

    tests = [
        test_health,
        test_chat_health,
        test_full_book_query,
        test_selected_text_query,
        test_sources,
        test_error_handling
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"Test failed with exception: {e}")
            results.append(False)

    print("\n" + "="*60)
    print("Test Results")
    print("="*60)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print(f"✗ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    exit_code = run_all_tests()
    sys.exit(exit_code)
