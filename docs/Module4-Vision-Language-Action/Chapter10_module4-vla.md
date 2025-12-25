---
sidebar_position: 4
title: "Module 4: Vision-Language-Action Integration - Whisper, GPT, Voice-to-Action, and ROS Actions"
---

# Module 4: Vision-Language-Action Integration - Whisper, GPT, Voice-to-Action, and ROS Actions

## Learning Objectives

By the end of this module, you will be able to:
- Integrate speech recognition using OpenAI Whisper for voice command processing
- Use GPT models for natural language understanding and planning
- Implement vision-language models for scene understanding
- Create voice-to-action systems that translate speech commands into robot actions
- Implement ROS Actions for long-running tasks with feedback
- Build complete vision-language-action pipelines for humanoid robots
- Design multimodal perception systems that combine vision and language

## Introduction to Vision-Language-Action Integration

Vision-Language-Action integration represents the cutting edge of robotics, where robots can understand natural language commands, perceive their environment visually, and execute complex actions. This integration enables robots to interact naturally with humans and perform tasks in unstructured environments.

### Key Components of Vision-Language-Action Systems

1. **Vision Processing**: Computer vision for scene understanding
2. **Language Processing**: Natural language understanding and generation
3. **Action Planning**: Converting high-level commands to low-level actions
4. **Execution**: Performing actions through robotic systems
5. **Feedback**: Sensory feedback to confirm action completion

## Speech Recognition with OpenAI Whisper

### Overview of Whisper for Robotics

OpenAI Whisper is a state-of-the-art speech recognition model that can be used for voice command processing in robotics applications. It provides robust speech-to-text capabilities that work well in noisy environments.

### Whisper Integration Node

Create the Whisper integration node `~/ai_vla_examples/ai_vla_examples/whisper_processor.py`:

```python
#!/usr/bin/env python3
"""
Whisper speech recognition node for voice command processing.
This node demonstrates integration of OpenAI Whisper with ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import torch
import whisper
import pyaudio
import wave
import threading
import queue
import time
import io
from collections import deque


class WhisperProcessor(Node):
    """
    A node that processes voice commands using OpenAI Whisper.
    """
    
    def __init__(self):
        # Initialize the node with the name 'whisper_processor'
        super().__init__('whisper_processor')
        
        # Create subscriptions
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10)
        
        # Create publishers
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.status_pub = self.create_publisher(String, 'whisper_status', 10)
        self.interpretation_pub = self.create_publisher(String, 'command_interpretation', 10)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize Whisper model
        self.get_logger().info('Loading Whisper model...')
        try:
            self.model = whisper.load_model("base")  # Use "base" for faster processing
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {str(e)}')
            self.model = None
        
        # Audio processing parameters
        self.audio_queue = queue.Queue()
        self.audio_buffer = deque(maxlen=16000 * 5)  # 5 seconds of audio at 16kHz
        self.sample_rate = 16000
        self.chunk_size = 1024
        
        # Voice command state
        self.listening = True
        self.command_history = deque(maxlen=20)
        
        # Create a timer for audio processing
        self.audio_timer = self.create_timer(0.1, self.process_audio)
        
        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.audio_processing_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        # Log that the Whisper processor has started
        self.get_logger().info('Whisper Processor has started')
    
    def audio_callback(self, msg):
        """
        Callback for audio data messages.
        """
        # Convert audio data to numpy array
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        
        # Add to buffer
        for sample in audio_data:
            self.audio_buffer.append(sample)
    
    def process_audio(self):
        """
        Process audio data and detect voice commands.
        """
        if not self.listening or self.model is None:
            return
        
        # Check if we have enough audio data for processing
        if len(self.audio_buffer) >= self.sample_rate * 2:  # At least 2 seconds
            # Convert buffer to numpy array
            audio_array = np.array(list(self.audio_buffer))
            
            # Check if there's significant audio activity (energy-based VAD)
            if self.is_speech_detected(audio_array):
                # Process the audio with Whisper
                threading.Thread(target=self.transcribe_audio, args=(audio_array,)).start()
                
                # Clear buffer after processing
                self.audio_buffer.clear()
    
    def is_speech_detected(self, audio_data):
        """
        Simple voice activity detection based on audio energy.
        """
        # Calculate energy of the audio signal
        energy = np.mean(np.abs(audio_data))
        
        # Threshold for speech detection (adjust based on environment)
        threshold = 0.01  # Adjust this value based on your microphone and environment
        
        return energy > threshold
    
    def transcribe_audio(self, audio_data):
        """
        Transcribe audio using Whisper model.
        """
        try:
            # Pad or trim audio to minimum length
            if len(audio_data) < self.sample_rate:
                # Pad with zeros
                padding = self.sample_rate - len(audio_data)
                audio_data = np.pad(audio_data, (0, padding), mode='constant')
            
            # Transcribe using Whisper
            result = self.model.transcribe(audio_data, fp16=False)
            text = result['text'].strip()
            
            if text:  # If transcription is not empty
                self.get_logger().info(f'Whisper transcription: "{text}"')
                
                # Publish the transcribed command
                command_msg = String()
                command_msg.data = text
                self.command_pub.publish(command_msg)
                
                # Add to command history
                self.command_history.append(text)
                
                # Interpret the command and publish interpretation
                interpretation = self.interpret_command(text)
                interpretation_msg = String()
                interpretation_msg.data = interpretation
                self.interpretation_pub.publish(interpretation_msg)
                
                # Publish status
                status_msg = String()
                status_msg.data = f'Command received: {text}'
                self.status_pub.publish(status_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error in Whisper transcription: {str(e)}')
    
    def interpret_command(self, command):
        """
        Interpret the voice command and extract intent.
        """
        command_lower = command.lower()
        
        # Define command patterns
        if 'move' in command_lower or 'go' in command_lower:
            if 'forward' in command_lower:
                return 'MOVE_FORWARD'
            elif 'backward' in command_lower:
                return 'MOVE_BACKWARD'
            elif 'left' in command_lower:
                return 'TURN_LEFT'
            elif 'right' in command_lower:
                return 'TURN_RIGHT'
            else:
                return 'MOVE_GENERIC'
        
        elif 'stop' in command_lower or 'halt' in command_lower:
            return 'STOP'
        
        elif 'pick' in command_lower or 'grasp' in command_lower:
            return 'PICK_OBJECT'
        
        elif 'place' in command_lower or 'put' in command_lower:
            return 'PLACE_OBJECT'
        
        elif 'find' in command_lower or 'look' in command_lower:
            return 'SEARCH_OBJECT'
        
        elif 'follow' in command_lower:
            return 'FOLLOW_HUMAN'
        
        elif 'come' in command_lower or 'here' in command_lower:
            return 'COME_HERE'
        
        else:
            return 'UNKNOWN_COMMAND'
    
    def audio_processing_loop(self):
        """
        Separate thread for audio processing to avoid blocking ROS.
        """
        while rclpy.ok():
            time.sleep(0.01)  # Small sleep to prevent busy waiting


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the WhisperProcessor node
    whisper_processor = WhisperProcessor()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(whisper_processor)
    except KeyboardInterrupt:
        whisper_processor.get_logger().info('Shutting down Whisper processor...')
    finally:
        # Destroy the node explicitly
        whisper_processor.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## GPT Integration for Natural Language Understanding

### Overview of GPT for Robotics

GPT models can be used for natural language understanding, task planning, and generating appropriate responses for robotic systems. They help bridge the gap between human language and robot actions.

### GPT Integration Node

Create the GPT integration node `~/ai_vla_examples/ai_vla_examples/gpt_planner.py`:

```python
#!/usr/bin/env python3
"""
GPT-based natural language understanding and planning node.
This node demonstrates integration of GPT models with ROS 2 for task planning.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import openai
import json
import re
import time
from typing import Dict, List, Tuple


class GPTPlanner(Node):
    """
    A node that uses GPT for natural language understanding and task planning.
    """
    
    def __init__(self):
        # Initialize the node with the name 'gpt_planner'
        super().__init__('gpt_planner')
        
        # Create subscriptions
        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10)
        
        self.interpretation_sub = self.create_subscription(
            String,
            'command_interpretation',
            self.interpretation_callback,
            10)
        
        # Create publishers
        self.plan_pub = self.create_publisher(String, 'task_plan', 10)
        self.action_pub = self.create_publisher(String, 'robot_action', 10)
        self.response_pub = self.create_publisher(String, 'gpt_response', 10)
        
        # Initialize GPT client
        # Note: You'll need to set your OpenAI API key in environment variables
        # os.environ["OPENAI_API_KEY"] = "your-api-key-here"
        try:
            # Use a mock client for demonstration purposes
            # In practice, you would initialize the OpenAI client:
            # self.client = openai.OpenAI()
            self.gpt_available = True
            self.get_logger().info('GPT planner initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPT: {str(e)}')
            self.gpt_available = False
        
        # Initialize state
        self.current_command = ""
        self.current_interpretation = ""
        self.task_plan = []
        self.command_history = []
        
        # Define object locations (in a real system, this would come from perception)
        self.object_locations = {
            'coffee': Point(x=2.0, y=1.0, z=0.0),
            'book': Point(x=3.0, y=2.0, z=0.0),
            'ball': Point(x=1.0, y=3.0, z=0.0),
            'cup': Point(x=2.5, y=0.5, z=0.0)
        }
        
        # Define room locations
        self.room_locations = {
            'kitchen': Point(x=4.0, y=4.0, z=0.0),
            'living room': Point(x=0.0, y=0.0, z=0.0),
            'bedroom': Point(x=-2.0, y=2.0, z=0.0),
            'office': Point(x=3.0, y=-1.0, z=0.0)
        }
        
        # Log that the GPT planner has started
        self.get_logger().info('GPT Planner has started')
    
    def command_callback(self, msg):
        """
        Callback for voice command messages.
        """
        self.current_command = msg.data
        self.get_logger().info(f'Received command: {self.current_command}')
        
        # Process the command with GPT
        self.process_command_with_gpt(self.current_command)
    
    def interpretation_callback(self, msg):
        """
        Callback for command interpretation messages.
        """
        self.current_interpretation = msg.data
        self.get_logger().info(f'Command interpretation: {self.current_interpretation}')
    
    def process_command_with_gpt(self, command):
        """
        Process the command using GPT to generate a task plan.
        """
        if not self.gpt_available:
            # Use mock processing for demonstration
            plan = self.mock_gpt_processing(command)
        else:
            # Use actual GPT processing
            plan = self.actual_gpt_processing(command)
        
        # Store the plan
        self.task_plan = plan
        
        # Publish the plan
        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)
        
        # Execute the first action in the plan
        if plan:
            self.execute_action(plan[0])
        
        self.get_logger().info(f'Generated plan: {plan}')
    
    def mock_gpt_processing(self, command):
        """
        Mock GPT processing for demonstration purposes.
        """
        command_lower = command.lower()
        
        # Define command patterns and corresponding actions
        if 'move to' in command_lower or 'go to' in command_lower:
            # Extract location from command
            location = self.extract_location(command_lower)
            if location:
                return [
                    {
                        'action': 'navigate',
                        'target': location,
                        'description': f'Navigating to {location}'
                    }
                ]
        
        elif 'pick up' in command_lower or 'get' in command_lower:
            # Extract object from command
            obj = self.extract_object(command_lower)
            if obj:
                return [
                    {
                        'action': 'navigate',
                        'target': self.object_locations.get(obj, Point(x=0, y=0, z=0)),
                        'description': f'Navigating to {obj}'
                    },
                    {
                        'action': 'pick_object',
                        'object': obj,
                        'description': f'Picking up {obj}'
                    }
                ]
        
        elif 'bring' in command_lower and 'to' in command_lower:
            # Extract object and destination
            obj = self.extract_object(command_lower)
            destination = self.extract_location(command_lower)
            if obj and destination:
                return [
                    {
                        'action': 'navigate',
                        'target': self.object_locations.get(obj, Point(x=0, y=0, z=0)),
                        'description': f'Navigating to {obj}'
                    },
                    {
                        'action': 'pick_object',
                        'object': obj,
                        'description': f'Picking up {obj}'
                    },
                    {
                        'action': 'navigate',
                        'target': self.room_locations.get(destination, Point(x=0, y=0, z=0)),
                        'description': f'Navigating to {destination} with {obj}'
                    },
                    {
                        'action': 'place_object',
                        'description': f'Placing {obj} at destination'
                    }
                ]
        
        elif 'find' in command_lower:
            obj = self.extract_object(command_lower)
            if obj:
                return [
                    {
                        'action': 'search_object',
                        'object': obj,
                        'description': f'Searching for {obj}'
                    }
                ]
        
        # Default response for unrecognized commands
        return [
            {
                'action': 'unknown',
                'description': f'Command not understood: {command}',
                'response': 'I don\'t understand that command. Can you rephrase it?'
            }
        ]
    
    def actual_gpt_processing(self, command):
        """
        Actual GPT processing (commented out to avoid API calls without key).
        """
        '''
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": """You are a helpful assistant for a robot. 
                        Convert human commands into structured robot actions. 
                        Respond with a JSON array of actions. 
                        Each action should have: 'action', 'description', and any relevant parameters.
                        Actions can be: navigate, pick_object, place_object, search_object, follow_human, etc."""
                    },
                    {
                        "role": "user",
                        "content": f"Convert this command to robot actions: {command}"
                    }
                ],
                temperature=0.3,
                max_tokens=200
            )
            
            # Parse the response
            content = response.choices[0].message.content
            # Extract JSON from response
            json_match = re.search(r'\[.*\]', content, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group())
                return plan
            else:
                return [{'action': 'unknown', 'description': 'Could not parse GPT response'}]
        
        except Exception as e:
            self.get_logger().error(f'Error in GPT processing: {str(e)}')
            return [{'action': 'error', 'description': 'GPT processing error'}]
        '''
        # For now, use mock processing
        return self.mock_gpt_processing(command)
    
    def extract_object(self, command):
        """
        Extract object name from command.
        """
        for obj in self.object_locations.keys():
            if obj in command:
                return obj
        return None
    
    def extract_location(self, command):
        """
        Extract location name from command.
        """
        for location in self.room_locations.keys():
            if location in command:
                return location
        return None
    
    def execute_action(self, action):
        """
        Execute the specified action.
        """
        action_msg = String()
        action_msg.data = json.dumps(action)
        self.action_pub.publish(action_msg)
        
        # Generate response based on action
        response = self.generate_response(action)
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        
        self.get_logger().info(f'Executing action: {action["description"]}')
    
    def generate_response(self, action):
        """
        Generate a response based on the action.
        """
        if action['action'] == 'navigate':
            return f"Okay, I'm navigating to the {action['description'].split()[-1]}."
        elif action['action'] == 'pick_object':
            return f"Okay, I'm picking up the {action['object']}."
        elif action['action'] == 'place_object':
            return "Okay, I've placed the object."
        elif action['action'] == 'search_object':
            return f"Okay, I'm searching for the {action['object']}."
        elif action['action'] == 'unknown':
            return action.get('response', "I don't understand that command.")
        else:
            return f"Okay, I'm performing: {action['description']}."


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the GPTPlanner node
    gpt_planner = GPTPlanner()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(gpt_planner)
    except KeyboardInterrupt:
        gpt_planner.get_logger().info('Shutting down GPT planner...')
    finally:
        # Destroy the node explicitly
        gpt_planner.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Vision-Language Models for Scene Understanding

### Overview of Vision-Language Models

Vision-Language models combine computer vision and natural language processing to understand scenes in context. Models like CLIP (Contrastive Language-Image Pretraining) can be used for object recognition, scene understanding, and visual question answering.

### Vision-Language Integration Node

Create the Vision-Language integration node `~/ai_vla_examples/ai_vla_examples/vision_language_processor.py`:

```python
#!/usr/bin/env python3
"""
Vision-Language processing node using multimodal AI models.
This node demonstrates integration of vision and language processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import deque
import time
import json


class VisionLanguageProcessor(Node):
    """
    A node that processes vision and language together for scene understanding.
    """
    
    def __init__(self):
        # Initialize the node with the name 'vision_language_processor'
        super().__init__('vision_language_processor')
        
        # Create subscriptions
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10)
        
        self.query_sub = self.create_subscription(
            String,
            'vision_query',
            self.query_callback,
            10)
        
        # Create publishers
        self.scene_description_pub = self.create_publisher(String, 'scene_description', 10)
        self.object_locations_pub = self.create_publisher(String, 'object_locations', 10)
        self.vqa_response_pub = self.create_publisher(String, 'vqa_response', 10)
        self.detection_pub = self.create_publisher(String, 'object_detections', 10)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize vision processing parameters
        self.current_image = None
        self.current_command = ""
        self.current_query = ""
        self.object_detections = []
        self.scene_description = ""
        
        # Predefined object classes for demonstration
        self.object_classes = [
            'person', 'bottle', 'cup', 'chair', 'couch', 'potted plant',
            'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
            'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]
        
        # Create a timer for periodic processing
        self.processing_timer = self.create_timer(1.0, self.periodic_processing)
        
        # Log that the vision-language processor has started
        self.get_logger().info('Vision-Language Processor has started')
    
    def image_callback(self, msg):
        """
        Callback for image messages.
        """
        try:
            # Convert ROS Image to OpenCV image
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform object detection (mock implementation for demonstration)
            self.object_detections = self.mock_object_detection(self.current_image)
            
            # Publish object detections
            detections_msg = String()
            detections_msg.data = json.dumps(self.object_detections)
            self.detection_pub.publish(detections_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def command_callback(self, msg):
        """
        Callback for voice command messages.
        """
        self.current_command = msg.data
        self.get_logger().info(f'Received command for vision processing: {self.current_command}')
    
    def query_callback(self, msg):
        """
        Callback for vision query messages.
        """
        self.current_query = msg.data
        self.get_logger().info(f'Received vision query: {self.current_query}')
        
        # Process the query if we have an image
        if self.current_image is not None:
            response = self.process_vision_query(self.current_image, self.current_query)
            response_msg = String()
            response_msg.data = response
            self.vqa_response_pub.publish(response_msg)
    
    def mock_object_detection(self, image):
        """
        Mock object detection for demonstration purposes.
        In a real implementation, this would use YOLO, Detectron2, or similar.
        """
        height, width = image.shape[:2]
        
        # Generate mock detections based on image properties
        detections = []
        
        # Add some mock objects with random positions
        for i in range(3):  # Add 3 random objects
            obj_class = np.random.choice(self.object_classes)
            x = np.random.uniform(0.1, 0.9) * width
            y = np.random.uniform(0.2, 0.8) * height
            w = np.random.uniform(0.05, 0.15) * width
            h = np.random.uniform(0.05, 0.15) * height
            
            detection = {
                'class': obj_class,
                'bbox': [float(x), float(y), float(w), float(h)],
                'confidence': float(np.random.uniform(0.7, 0.95))
            }
            detections.append(detection)
        
        # Add a person if the image has human-like features
        if np.mean(image) >  # Simple # Simple # Simple heuristic
            x = width * 0.5
            y = height * 0.4
            w = width * 0.1
            h = height * 0.4
            detections.append({
                'class': 'person',
                'bbox': [float(x), float(y), float(w), float(h)],
                'confidence': 0.9
            })
        
        return detections
    
    def process_vision_query(self, image, query):
        """
        Process a vision query using the current image.
        """
        # Mock implementation of vision-language processing
        query_lower = query.lower()
        
        if 'where is' in query_lower or 'locate' in query_lower:
            # Find objects mentioned in the query
            for obj_class in self.object_classes:
                if obj_class in query_lower:
                    for detection in self.object_detections:
                        if detection['class'] == obj_class:
                            x, y, w, h = detection['bbox']
                            # Convert to relative position
                            pos_desc = self.get_position_description(x, y, image.shape[1], image.shape[0])
                            return f"The {obj_class} is {pos_desc} of the image."
            
            return f"I couldn't find the object you're looking for in the current view."
        
        elif 'how many' in query_lower or 'count' in query_lower:
            # Count objects of specific type
            for obj_class in self.object_classes:
                if obj_class in query_lower:
                    count = sum(1 for det in self.object_detections if det['class'] == obj_class)
                    return f"I can see {count} {obj_class}{'s' if count != 1 else ''} in the image."
            
            # Count all objects
            return f"I can see {len(self.object_detections)} objects in total."
        
        elif 'describe' in query_lower or 'what do you see' in query_lower:
            # Describe the scene
            return self.describe_scene()
        
        else:
            # Default response
            return "I can help with object location, counting, and scene description. Try asking 'Where is the chair?' or 'How many bottles do you see?'"
    
    def get_position_description(self, x, y, img_width, img_height):
        """
        Convert coordinates to position description (left/right, top/bottom).
        """
        horizontal_pos = "left" if x < img_width / 2 else "right"
        vertical_pos = "top" if y < img_height / 2 else "bottom"
        
        if horizontal_pos == "left" and vertical_pos == "top":
            return "in the top-left"
        elif horizontal_pos == "right" and vertical_pos == "top":
            return "in the top-right"
        elif horizontal_pos == "left" and vertical_pos == "bottom":
            return "in the bottom-left"
        else:
            return "in the bottom-right"
    
    def describe_scene(self):
        """
        Generate a description of the current scene.
        """
        if not self.object_detections:
            return "I don't see any objects clearly in the current view."
        
        # Count objects by class
        class_counts = {}
        for det in self.object_detections:
            obj_class = det['class']
            if obj_class in class_counts:
                class_counts[obj_class] += 1
            else:
                class_counts[obj_class] = 1
        
        # Generate description
        descriptions = []
        for obj_class, count in class_counts.items():
            if count == 1:
                descriptions.append(f"a {obj_class}")
            else:
                descriptions.append(f"{count} {obj_class}s")
        
        if len(descriptions) == 1:
            scene_desc = f"I see {descriptions[0]} in the scene."
        elif len(descriptions) == 2:
            scene_desc = f"I see {descriptions[0]} and {descriptions[1]} in the scene."
        else:
            scene_desc = f"I see {', '.join(descriptions[:-1])}, and {descriptions[-1]} in the scene."
        
        return scene_desc
    
    def periodic_processing(self):
        """
        Periodic processing for scene understanding.
        """
        if self.current_image is not None:
            # Update scene description periodically
            self.scene_description = self.describe_scene()
            
            # Publish scene description
            desc_msg = String()
            desc_msg.data = self.scene_description
            self.scene_description_pub.publish(desc_msg)
            
            # Publish object locations
            locations_msg = String()
            locations_msg.data = json.dumps(self.get_object_locations())
            self.object_locations_pub.publish(locations_msg)
    
    def get_object_locations(self):
        """
        Get object locations in a structured format.
        """
        locations = {}
        for detection in self.object_detections:
            obj_class = detection['class']
            x, y, w, h = detection['bbox']
            pos_desc = self.get_position_description(x, y, self.current_image.shape[1], self.current_image.shape[0])
            
            if obj_class not in locations:
                locations[obj_class] = []
            
            locations[obj_class].append({
                'position': pos_desc,
                'confidence': detection['confidence']
            })
        
        return locations


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the VisionLanguageProcessor node
    vision_language_processor = VisionLanguageProcessor()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(vision_language_processor)
    except KeyboardInterrupt:
        vision_language_processor.get_logger().info('Shutting down Vision-Language processor...')
    finally:
        # Destroy the node explicitly
        vision_language_processor.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Voice-to-Action System

### Overview of Voice-to-Action Pipeline

The voice-to-action system converts natural language commands into specific robot actions through a pipeline of speech recognition, natural language understanding, task planning, and action execution.

### Voice-to-Action Integration Node

Create the Voice-to-Action integration node `~/ai_vla_examples/ai_vla_examples/voice_to_action.py`:

```python
#!/usr/bin/env python3
"""
Voice-to-Action system that converts voice commands to robot actions.
This node integrates speech recognition, NLU, and action execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point
import json
import math
import time
from enum import Enum


class RobotState(Enum):
    IDLE = 1
    LISTENING = 2
    PROCESSING = 3
    EXECUTING = 4
    ERROR = 5


class VoiceToAction(Node):
    """
    A node that converts voice commands to robot actions.
    """
    
    def __init__(self):
        # Initialize the node with the name 'voice_to_action'
        super().__init__('voice_to_action')
        
        # Create subscriptions
        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10)
        
        self.interpretation_sub = self.create_subscription(
            String,
            'command_interpretation',
            self.interpretation_callback,
            10)
        
        self.plan_sub = self.create_subscription(
            String,
            'task_plan',
            self.plan_callback,
            10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.action_status_pub = self.create_publisher(String, 'action_status', 10)
        
        # Create action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialize state
        self.current_state = RobotState.IDLE
        self.current_command = ""
        self.current_interpretation = ""
        self.current_plan = []
        self.current_step = 0
        self.obstacle_detected = False
        
        # Robot parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.safe_distance = 0.5
        
        # Navigation parameters
        self.current_goal = None
        self.navigation_active = False
        
        # Create a timer for action execution
        self.action_timer = self.create_timer(0.1, self.execute_action)
        
        # Log that the voice-to-action system has started
        self.get_logger().info('Voice-to-Action system has started')
    
    def command_callback(self, msg):
        """
        Callback for voice command messages.
        """
        self.current_command = msg.data
        self.current_state = RobotState.PROCESSING
        
        status_msg = String()
        status_msg.data = f'Processing command: {self.current_command}'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f'Received voice command: {self.current_command}')
    
    def interpretation_callback(self, msg):
        """
        Callback for command interpretation messages.
        """
        self.current_interpretation = msg.data
        self.get_logger().info(f'Command interpretation: {self.current_interpretation}')
    
    def plan_callback(self, msg):
        """
        Callback for task plan messages.
        """
        try:
            self.current_plan = json.loads(msg.data)
            self.current_step = 0
            self.current_state = RobotState.EXECUTING
            
            status_msg = String()
            status_msg.data = f'Executing plan with {len(self.current_plan)} steps'
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f'Received task plan with {len(self.current_plan)} steps')
        except json.JSONDecodeError:
            self.get_logger().error('Failed to decode task plan')
    
    def scan_callback(self, msg):
        """
        Callback for laser scan messages to detect obstacles.
        """
        if len(msg.ranges) > 0:
            front_ranges = msg.ranges[len(msg.ranges)//2 - 30 : len(msg.ranges)//2 + 30]
            front_min = min(r for r in front_ranges if not math.isnan(r) and r > 0)
            self.obstacle_detected = front_min < self.safe_distance
    
    def execute_action(self):
        """
        Execute the current action in the plan.
        """
        if not self.current_plan or self.current_step >= len(self.current_plan):
            if self.current_state == RobotState.EXECUTING:
                self.current_state = RobotState.IDLE
                status_msg = String()
                status_msg.data = 'Plan execution completed'
                self.status_pub.publish(status_msg)
            return
        
        # Check for obstacles during navigation
        if self.navigation_active and self.obstacle_detected:
            self.emergency_stop()
            status_msg = String()
            status_msg.data = 'Obstacle detected, navigation paused'
            self.status_pub.publish(status_msg)
            return
        
        # Execute the current step
        current_action = self.current_plan[self.current_step]
        
        action_status_msg = String()
        action_status_msg.data = f'Executing: {current_action["description"]}'
        self.action_status_pub.publish(action_status_msg)
        
        success = self.perform_action(current_action)
        
        if success:
            self.current_step += 1
            if self.current_step >= len(self.current_plan):
                self.current_state = RobotState.IDLE
                status_msg = String()
                status_msg.data = 'All actions completed successfully'
                self.status_pub.publish(status_msg)
    
    def perform_action(self, action):
        """
        Perform a specific action based on its type.
        """
        action_type = action['action']
        
        if action_type == 'navigate':
            return self.navigate_to_pose(action)
        elif action_type == 'move_forward':
            return self.move_forward()
        elif action_type == 'turn_left':
            return self.turn_left()
        elif action_type == 'turn_right':
            return self.turn_right()
        elif action_type == 'stop':
            return self.stop_robot()
        elif action_type == 'pick_object':
            return self.pick_object()
        elif action_type == 'place_object':
            return self.place_object()
        elif action_type == 'search_object':
            return self.search_object()
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return True  # Consider unknown actions as completed
    
    def navigate_to_pose(self, action):
        """
        Navigate to a specific pose.
        """
        if 'target' in action:
            target = action['target']
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = target.get('x', 0.0)
            pose_msg.pose.position.y = target.get('y', 0.0)
            pose_msg.pose.position.z = target.get('z', 0.0)
            pose_msg.pose.orientation.w = 1.0  # Default orientation
            
            # Send navigation goal
            return self.send_navigation_goal(pose_msg.pose)
        
        return False
    
    def send_navigation_goal(self, pose):
        """
        Send navigation goal to Nav2.
        """
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Send goal asynchronously
        self.nav_to_pose_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback)
        
        self.nav_to_pose_future.add_done_callback(self.navigation_goal_response_callback)
        
        self.navigation_active = True
        self.get_logger().info(f'Sent navigation goal to ({pose.position.x}, {pose.position.y})')
        
        return True
    
    def navigation_goal_response_callback(self, future):
        """
        Handle navigation goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal was rejected')
            self.navigation_active = False
            return
        
        self.get_logger().info('Navigation goal accepted, executing...')
        
        # Get result future
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """
        Handle navigation result.
        """
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # Goal succeeded
            self.get_logger().info('Navigation goal succeeded')
            self.navigation_active = False
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
            self.navigation_active = False
    
    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.current_pose}')
    
    def move_forward(self):
        """
        Move robot forward.
        """
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
        # Stop after a short time (in a real system, this would be based on distance)
        time.sleep(1.0)
        
        self.stop_robot()
        return True
    
    def turn_left(self):
        """
        Turn robot left.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist_msg)
        
        # Turn for a short time
        time.sleep(0.5)
        
        self.stop_robot()
        return True
    
    def turn_right(self):
        """
        Turn robot right.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(twist_msg)
        
        # Turn for a short time
        time.sleep(0.5)
        
        self.stop_robot()
        return True
    
    def stop_robot(self):
        """
        Stop the robot immediately.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        return True
    
    def pick_object(self):
        """
        Simulate picking an object.
        """
        # In a real robot, this would involve arm control
        self.get_logger().info('Simulating object pickup')
        time.sleep(2.0)  # Simulate time for picking
        return True
    
    def place_object(self):
        """
        Simulate placing an object.
        """
        # In a real robot, this would involve arm control
        self.get_logger().info('Simulating object placement')
        time.sleep(2.0)  # Simulate time for placing
        return True
    
    def search_object(self):
        """
        Simulate searching for an object.
        """
        # In a real robot, this would involve systematic scanning
        self.get_logger().info('Simulating object search')
        time.sleep(3.0)  # Simulate time for searching
        return True
    
    def emergency_stop(self):
        """
        Emergency stop due to obstacle detection.
        """
        self.stop_robot()
        self.navigation_active = False
        
        # In a real system, you might want to replan or wait
        status_msg = String()
        status_msg.data = 'Emergency stop - obstacle detected'
        self.status_pub.publish(status_msg)


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the VoiceToAction node
    voice_to_action = VoiceToAction()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(voice_to_action)
    except KeyboardInterrupt:
        voice_to_action.get_logger().info('Shutting down Voice-to-Action system...')
    finally:
        # Destroy the node explicitly
        voice_to_action.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## ROS Actions for Long-Running Tasks

### Overview of ROS Actions

ROS Actions are used for long-running tasks that provide feedback during execution and can be canceled. They're perfect for navigation, manipulation, and other extended robot behaviors.

### Custom Action Definition

First, let's create a custom action definition. Create the directory and action file:

```bash
mkdir -p ~/ai_vla_examples/ai_vla_examples/action
```

Create the action file `~/ai_vla_examples/ai_vla_examples/action/RobotTask.action`:

```
# Goal definition
string task_type  # "navigation", "manipulation", "inspection", etc.
float64[] target_position  # [x, y, z] coordinates
string object_name  # Name of object for manipulation tasks
string description  # Human-readable task description
---
# Result definition
bool success
string message
float64 execution_time
string final_status
---
# Feedback definition
string current_status
float64 progress_percentage
float64 estimated_time_remaining
string current_action
```

### ROS Action Server

Create the action server `~/ai_vla_examples/ai_vla_examples/robot_task_server.py`:

```python
#!/usr/bin/env python3
"""
Robot task action server for handling complex robot behaviors.
This server handles long-running tasks with feedback and cancellation.
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from ai_vla_examples.action import RobotTask  # Import the custom action
import time
import math
import threading


class RobotTaskServer(Node):
    """
    An action server that handles complex robot tasks with feedback.
    """
    
    def __init__(self):
        # Initialize the node with the name 'robot_task_server'
        super().__init__('robot_task_server')
        
        # Create an action server for the RobotTask action
        self._action_server = ActionServer(
            self,
            RobotTask,
            'execute_robot_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        # Create publishers and subscribers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Initialize state
        self.obstacle_detected = False
        self.current_task = None
        self.task_cancel_requested = False
        
        # Log that the action server has started
        self.get_logger().info('Robot Task Action Server has started')
    
    def goal_callback(self, goal_request):
        """
        Accept or reject a client request to begin an action.
        """
        self.get_logger().info(f'Received task goal: {goal_request.task_type}')
        
        # Accept all goals for this example
        # In a real system, you might have conditions for acceptance
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """
        Accept or reject a client request to cancel an action.
        """
        self.get_logger().info('Received cancel request')
        self.task_cancel_requested = True
        return CancelResponse.ACCEPT
    
    def scan_callback(self, msg):
        """
        Callback for laser scan to detect obstacles during task execution.
        """
        if len(msg.ranges) > 0:
            front_ranges = msg.ranges[len(msg.ranges)//2 - 30 : len(msg.ranges)//2 + 30]
            front_min = min(r for r in front_ranges if not math.isnan(r) and r > 0)
            self.obstacle_detected = front_min < 0.5  # 0.5m safety distance
    
    def execute_callback(self, goal_handle):
        """
        Execute the robot task goal.
        """
        self.get_logger().info('Executing robot task goal...')
        
        # Store the current task
        self.current_task = goal_handle
        self.task_cancel_requested = False
        
        # Get the task type and parameters
        task_type = goal_handle.request.task_type
        target_position = goal_handle.request.target_position
        object_name = goal_handle.request.object_name
        description = goal_handle.request.description
        
        self.get_logger().info(f'Task: {task_type}, Target: {target_position}, Object: {object_name}')
        
        # Initialize feedback
        feedback_msg = RobotTask.Feedback()
        feedback_msg.current_status = f'Starting {task_type} task'
        feedback_msg.progress_percentage = 0.0
        feedback_msg.estimated_time_remaining = 0.0
        feedback_msg.current_action = 'initializing'
        
        # Publish initial feedback
        goal_handle.publish_feedback(feedback_msg)
        
        # Execute the task based on type
        success = False
        message = ""
        
        if task_type == 'navigation':
            success = self.execute_navigation_task(target_position, goal_handle, feedback_msg)
        elif task_type == 'manipulation':
            success = self.execute_manipulation_task(object_name, goal_handle, feedback_msg)
        elif task_type == 'inspection':
            success = self.execute_inspection_task(goal_handle, feedback_msg)
        else:
            success = False
            message = f'Unknown task type: {task_type}'
        
        # Prepare result
        result = RobotTask.Result()
        result.success = success
        result.message = message if message else f'Task {task_type} completed successfully'
        result.execution_time = time.time()  # In a real system, track actual execution time
        result.final_status = 'completed' if success else 'failed'
        
        if goal_handle.is_canceling():
            goal_handle.canceled()
            result.success = False
            result.message = 'Task was canceled'
            result.final_status = 'canceled'
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        self.get_logger().info(f'Task result: {result.message}')
        return result
    
    def execute_navigation_task(self, target_position, goal_handle, feedback_msg):
        """
        Execute a navigation task.
        """
        if len(target_position) < 3:
            return False
        
        target_x, target_y, target_z = target_position
        
        # Calculate distance to target
        # In a real system, you'd get current position from odometry
        current_x, current_y = 0.0, 0.0  # Starting position
        total_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Simulate navigation
        steps = 50
        for i in range(steps):
            if goal_handle.is_canceling():
                return False
            
            # Check for obstacles
            if self.obstacle_detected:
                feedback_msg.current_status = 'Obstacle detected, pausing navigation'
                feedback_msg.current_action = 'avoiding_obstacle'
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)  # Wait before continuing
                continue
            
            # Update progress
            progress = (i + 1) / steps * 100
            feedback_msg.progress_percentage = progress
            feedback_msg.current_status = f'Navigating to ({target_x}, {target_y})'
            feedback_msg.current_action = 'moving'
            feedback_msg.estimated_time_remaining = (steps - i - 1) * 0.1  # 0.1s per step
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate movement (in a real system, send commands to navigation stack)
            time.sleep(0.1)
        
        return True
    
    def execute_manipulation_task(self, object_name, goal_handle, feedback_msg):
        """
        Execute a manipulation task.
        """
        if not object_name:
            return False
        
        # Simulate manipulation sequence
        actions = [
            ('approaching_object', 'Moving to object position'),
            ('aligning_gripper', 'Aligning gripper with object'),
            ('grasping', 'Grasping the object'),
            ('lifting', 'Lifting the object'),
            ('transporting', 'Transporting object to destination'),
            ('placing', 'Placing object at destination'),
            ('retracting', 'Retracting manipulator')
        ]
        
        for i, (action, description) in enumerate(actions):
            if goal_handle.is_canceling():
                return False
            
            feedback_msg.current_status = description
            feedback_msg.current_action = action
            feedback_msg.progress_percentage = (i + 1) / len(actions) * 100
            feedback_msg.estimated_time_remaining = (len(actions) - i - 1) * 2.0  # 2s per action
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate action time
            time.sleep(2.0)
        
        return True
    
    def execute_inspection_task(self, goal_handle, feedback_msg):
        """
        Execute an inspection task.
        """
        # Simulate inspection sequence
        inspection_points = [
            (0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)  # Square pattern
        ]
        
        for i, point in enumerate(inspection_points):
            if goal_handle.is_canceling():
                return False
            
            feedback_msg.current_status = f'Inspecting area {i+1} of {len(inspection_points)}'
            feedback_msg.current_action = f'inspecting_point_{i+1}'
            feedback_msg.progress_percentage = (i + 1) / len(inspection_points) * 100
            feedback_msg.estimated_time_remaining = (len(inspection_points) - i - 1) * 3.0
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate inspection time
            time.sleep(3.0)
        
        return True


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the RobotTaskServer node
    robot_task_server = RobotTaskServer()
    
    # Spin the node so the action callbacks are called
    rclpy.spin(robot_task_server)
    
    # Destroy the node explicitly
    robot_task_server.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### ROS Action Client

Create the action client `~/ai_vla_examples/ai_vla_examples/robot_task_client.py`:

```python
#!/usr/bin/env python3
"""
Robot task action client for requesting complex robot behaviors.
This client demonstrates how to use the RobotTask action.
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ai_vla_examples.action import RobotTask  # Import the custom action


class RobotTaskClient(Node):
    """
    An action client that requests robot tasks.
    """
    
    def __init__(self):
        # Initialize the node with the name 'robot_task_client'
        super().__init__('robot_task_client')
        
        # Create an action client for the RobotTask action
        self._action_client = ActionClient(self, RobotTask, 'execute_robot_task')
    
    def send_task(self, task_type, target_position=None, object_name="", description=""):
        """
        Send a robot task to the action server.
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server to be available...')
        self._action_client.wait_for_server()
        
        # Create a goal message
        goal_msg = RobotTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.target_position = target_position if target_position else []
        goal_msg.object_name = object_name
        goal_msg.description = description
        
        # Send the goal and get a future for the result
        self.get_logger().info(f'Sending {task_type} task goal')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        # Add a callback to handle the result
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def goal_response_callback(self, future):
        """
        Handle the goal response from the action server.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the action server')
            return
        
        self.get_logger().info('Goal was accepted by the action server, waiting for result...')
        
        # Get the result future
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Handle the result from the action server.
        """
        result = future.result().result
        self.get_logger().info(f'Result: Success={result.success}, Message="{result.message}"')
        self.get_logger().info(f'Execution time: {result.execution_time:.2f}s')
        self.get_logger().info(f'Final status: {result.final_status}')
    
    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server during execution.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.current_status} - '
            f'Progress: {feedback.progress_percentage:.1f}% - '
            f'Action: {feedback.current_action}'
        )


def main(args=None):
    """
    Main function that initializes the node and sends action goals.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the RobotTaskClient node
    action_client = RobotTaskClient()
    
    # Check if command line arguments are provided
    if len(sys.argv) < 2:
        print('Usage: ros2 run ai_vla_examples robot_task_client <task_type> [target_x] [target_y] [target_z] [object_name]')
        print('Example: ros2 run ai_vla_examples robot_task_client navigation 5.0 5.0 0.0')
        print('Example: ros2 run ai_vla_examples robot_task_client manipulation "" "" "" "red_cup"')
        return
    
    # Parse command line arguments
    task_type = sys.argv[1]
    
    if task_type == 'navigation' and len(sys.argv) >= 5:
        try:
            target_x = float(sys.argv[2])
            target_y = float(sys.argv[3])
            target_z = float(sys.argv[4])
            target_position = [target_x, target_y, target_z]
            object_name = ""
            description = f"Navigate to position ({target_x}, {target_y}, {target_z})"
        except ValueError:
            print('Error: Please provide valid floating point numbers for coordinates')
            return
    elif task_type == 'manipulation' and len(sys.argv) >= 3:
        target_position = []
        object_name = sys.argv[2]
        description = f"Manipulate object: {object_name}"
    else:
        target_position = []
        object_name = ""
        description = f"Execute {task_type} task"
    
    # Send the robot task goal
    future = action_client.send_task(task_type, target_position, object_name, description)
    
    # Spin to handle callbacks
    rclpy.spin(action_client)
    
    # Destroy the node explicitly
    action_client.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Complete Vision-Language-Action Pipeline

### Main Integration Node

Create the main integration node `~/ai_vla_examples/ai_vla_examples/vla_main.py`:

```python
#!/usr/bin/env python3
"""
Main Vision-Language-Action integration node.
This node coordinates all components of the VLA system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from ai_vla_examples.action import RobotTask
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import json
import time
import threading


class VLAMain(Node):
    """
    Main node that integrates Vision, Language, and Action components.
    """
    
    def __init__(self):
        # Initialize the node with the name 'vla_main'
        super().__init__('vla_main')
        
        # Create subscriptions
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            10)
        
        self.scene_description_sub = self.create_subscription(
            String,
            'scene_description',
            self.scene_description_callback,
            10)
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        # Create publishers
        self.status_pub = self.create_publisher(String, 'vla_status', 10)
        self.system_response_pub = self.create_publisher(String, 'system_response', 10)
        
        # Initialize components
        self.bridge = CvBridge()
        self.current_image = None
        self.current_scene_description = ""
        self.system_active = True
        
        # Initialize action client for complex tasks
        self.task_action_client = ActionClient(self, RobotTask, 'execute_robot_task')
        
        # System state
        self.conversation_history = []
        self.object_memory = {}  # Remember object locations
        self.room_layout = {}    # Remember room layout
        
        # Create a timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.system_monitor)
        
        # Log that the VLA system has started
        self.get_logger().info('Vision-Language-Action system has started')
    
    def voice_command_callback(self, msg):
        """
        Callback for voice commands - main entry point for VLA system.
        """
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')
        
        # Add to conversation history
        self.conversation_history.append({
            'type': 'command',
            'text': command,
            'timestamp': time.time()
        })
        
        # Process the command through the VLA pipeline
        self.process_vla_command(command)
    
    def scene_description_callback(self, msg):
        """
        Callback for scene descriptions from vision system.
        """
        self.current_scene_description = msg.data
        self.get_logger().info(f'Updated scene description: {self.current_scene_description}')
    
    def image_callback(self, msg):
        """
        Callback for camera images.
        """
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_vla_command(self, command):
        """
        Process a command through the complete VLA pipeline.
        """
        # Step 1: Language Understanding
        intent = self.understand_language(command)
        
        # Step 2: Vision Processing (if needed)
        vision_info = self.process_vision(command)
        
        # Step 3: Action Planning
        action_plan = self.plan_actions(intent, vision_info, command)
        
        # Step 4: Action Execution
        self.execute_plan(action_plan, command)
    
    def understand_language(self, command):
        """
        Understand the intent behind the language command.
        """
        command_lower = command.lower()
        
        # Define command patterns
        if any(word in command_lower for word in ['go to', 'navigate to', 'move to', 'walk to']):
            return {
                'type': 'navigation',
                'target': self.extract_location(command_lower),
                'original_command': command
            }
        elif any(word in command_lower for word in ['pick up', 'get', 'bring me', 'grab']):
            return {
                'type': 'manipulation',
                'action': 'pick',
                'object': self.extract_object(command_lower),
                'original_command': command
            }
        elif any(word in command_lower for word in ['put down', 'place', 'set down']):
            return {
                'type': 'manipulation',
                'action': 'place',
                'object': self.extract_object(command_lower),
                'location': self.extract_location(command_lower),
                'original_command': command
            }
        elif any(word in command_lower for word in ['find', 'locate', 'where is', 'look for']):
            return {
                'type': 'search',
                'object': self.extract_object(command_lower),
                'original_command': command
            }
        elif any(word in command_lower for word in ['describe', 'what do you see', 'tell me about']):
            return {
                'type': 'describe',
                'original_command': command
            }
        elif any(word in command_lower for word in ['follow', 'come with', 'accompany']):
            return {
                'type': 'follow',
                'original_command': command
            }
        else:
            return {
                'type': 'unknown',
                'original_command': command
            }
    
    def extract_location(self, command):
        """
        Extract location from command.
        """
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'bathroom', 'dining room']
        for loc in locations:
            if loc in command:
                return loc
        return 'unknown'
    
    def extract_object(self, command):
        """
        Extract object from command.
        """
        # Common objects that might be in the environment
        objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'ball', 'toy', 'box', 'chair', 'table']
        for obj in objects:
            if obj in command:
                return obj
        return 'unknown'
    
    def process_vision(self, command):
        """
        Process vision information relevant to the command.
        """
        # In a real system, this would run object detection, scene analysis, etc.
        # For now, return mock vision information
        return {
            'scene_description': self.current_scene_description,
            'object_locations': self.object_memory,
            'room_layout': self.room_layout
        }
    
    def plan_actions(self, intent, vision_info, command):
        """
        Plan actions based on intent and vision information.
        """
        action_plan = []
        
        intent_type = intent['type']
        
        if intent_type == 'navigation':
            location = intent.get('target', 'unknown')
            if location != 'unknown':
                action_plan = [
                    {
                        'type': 'action',
                        'name': 'navigate_to_location',
                        'parameters': {'location': location},
                        'description': f'Navigating to {location}'
                    }
                ]
        
        elif intent_type == 'manipulation':
            action = intent.get('action', 'unknown')
            obj = intent.get('object', 'unknown')
            
            if action == 'pick' and obj != 'unknown':
                # First navigate to object, then pick it
                action_plan = [
                    {
                        'type': 'action',
                        'name': 'navigate_to_object',
                        'parameters': {'object': obj},
                        'description': f'Navigating to {obj}'
                    },
                    {
                        'type': 'action',
                        'name': 'pick_object',
                        'parameters': {'object': obj},
                        'description': f'Picking up {obj}'
                    }
                ]
        
        elif intent_type == 'search':
            obj = intent.get('object', 'unknown')
            if obj != 'unknown':
                action_plan = [
                    {
                        'type': 'action',
                        'name': 'search_for_object',
                        'parameters': {'object': obj},
                        'description': f'Searching for {obj}'
                    }
                ]
        
        elif intent_type == 'describe':
            action_plan = [
                {
                    'type': 'response',
                    'content': self.current_scene_description,
                    'description': 'Describing current scene'
                }
            ]
        
        else:
            action_plan = [
                {
                    'type': 'response',
                    'content': "I don't understand that command. Can you please rephrase it?",
                    'description': 'Unknown command response'
                }
            ]
        
        return action_plan
    
    def execute_plan(self, action_plan, original_command):
        """
        Execute the planned actions.
        """
        for action in action_plan:
            if action['type'] == 'action':
                success = self.execute_single_action(action)
                if not success:
                    self.get_logger().error(f'Action failed: {action["description"]}')
                    break
            elif action['type'] == 'response':
                response_msg = String()
                response_msg.data = action['content']
                self.system_response_pub.publish(response_msg)
                
                # Also publish to status
                status_msg = String()
                status_msg.data = action['description']
                self.status_pub.publish(status_msg)
    
    def execute_single_action(self, action):
        """
        Execute a single action.
        """
        action_name = action['name']
        params = action['parameters']
        
        self.get_logger().info(f'Executing action: {action["description"]}')
        
        # Publish action status
        status_msg = String()
        status_msg.data = action['description']
        self.status_pub.publish(status_msg)
        
        # In a real system, these would trigger actual robot behaviors
        if action_name == 'navigate_to_location':
            return self.execute_navigation_action(params)
        elif action_name == 'navigate_to_object':
            return self.execute_navigation_to_object(params)
        elif action_name == 'pick_object':
            return self.execute_pick_action(params)
        elif action_name == 'search_for_object':
            return self.execute_search_action(params)
        else:
            self.get_logger().warn(f'Unknown action: {action_name}')
            return False
    
    def execute_navigation_action(self, params):
        """
        Execute navigation action.
        """
        location = params.get('location', 'unknown')
        self.get_logger().info(f'Navigating to {location}')
        
        # In a real system, this would send navigation goals
        # For simulation, we'll just wait
        time.sleep(2.0)
        
        # Publish success
        response_msg = String()
        response_msg.data = f"I have reached the {location}."
        self.system_response_pub.publish(response_msg)
        
        return True
    
    def execute_navigation_to_object(self, params):
        """
        Execute navigation to object action.
        """
        obj = params.get('object', 'unknown')
        self.get_logger().info(f'Navigating to {obj}')
        
        # In a real system, this would use object locations from vision system
        # For simulation, we'll just wait
        time.sleep(2.0)
        
        return True
    
    def execute_pick_action(self, params):
        """
        Execute pick action.
        """
        obj = params.get('object', 'unknown')
        self.get_logger().info(f'Picking {obj}')
        
        # In a real system, this would control the robot's manipulator
        # For simulation, we'll just wait
        time.sleep(3.0)
        
        # Publish success
        response_msg = String()
        response_msg.data = f"I have picked up the {obj}."
        self.system_response_pub.publish(response_msg)
        
        return True
    
    def execute_search_action(self, params):
        """
        Execute search action.
        """
        obj = params.get('object', 'unknown')
        self.get_logger().info(f'Searching for {obj}')
        
        # In a real system, this would perform systematic search
        # For simulation, we'll just wait
        time.sleep(4.0)
        
        # Publish result
        response_msg = String()
        response_msg.data = f"I found the {obj} in the current area."
        self.system_response_pub.publish(response_msg)
        
        return True
    
    def system_monitor(self):
        """
        Monitor system status and health.
        """
        status_msg = String()
        status_msg.data = f'VLA System Active - Commands Processed: {len(self.conversation_history)}'
        self.status_pub.publish(status_msg)
    
    def shutdown(self):
        """
        Clean shutdown of the VLA system.
        """
        self.system_active = False
        self.get_logger().info('VLA system shutting down...')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the VLAMain node
    vla_main = VLAMain()
    
    try:
        # Spin the node so the callback functions are called
        rclpy.spin(vla_main)
    except KeyboardInterrupt:
        vla_main.get_logger().info('Shutting down VLA system...')
        vla_main.shutdown()
    finally:
        # Destroy the node explicitly
        vla_main.destroy_node()
        
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Create the setup.py file for the AI VLA examples package:

```python
from setuptools import find_packages, setup

package_name = 'ai_vla_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['ai_vla_examples/action/RobotTask.action']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'scipy', 'openai'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Vision-Language-Action integration examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_processor = ai_vla_examples.whisper_processor:main',
            'gpt_planner = ai_vla_examples.gpt_planner:main',
            'vision_language_processor = ai_vla_examples.vision_language_processor:main',
            'voice_to_action = ai_vla_examples.voice_to_action:main',
            'robot_task_server = ai_vla_examples.robot_task_server:main',
            'robot_task_client = ai_vla_examples.robot_task_client:main',
            'vla_main = ai_vla_examples.vla_main:main',
        ],
    },
)
```

Create the package.xml file:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ai_vla_examples</name>
  <version>0.0.0</version>
  <description>Vision-Language-Action integration examples</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>action_msgs</depend>
  <depend>rosidl_default_generators</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Exercises and Mini Projects

### Exercise 1: Multimodal Command Processing

Create a system that can handle commands combining vision and language, such as "Pick up the red cup on the table".

**Solution:**

This would involve creating a node that integrates object detection, color recognition, and spatial reasoning to understand and execute complex multimodal commands.

### Exercise 2: Conversational Robotics

Create a system that maintains context across multiple interactions, allowing for natural conversation with the robot.

**Solution:**

This would involve implementing a dialogue manager that tracks conversation state, maintains context, and allows for follow-up questions and commands.

### Mini Project: Complete VLA Robot Assistant

Create a complete system that integrates all components:
- Speech recognition for command input
- Vision processing for scene understanding
- Natural language processing for command interpretation
- Task planning for action sequencing
- Action execution for robot control
- Feedback and response generation

This project would demonstrate the complete pipeline from voice command to robot action execution, showcasing the full Vision-Language-Action integration.

## Summary

In this module, we've covered:

1. **Speech Recognition**: Integration of OpenAI Whisper for voice command processing
2. **Natural Language Understanding**: Using GPT models for command interpretation and planning
3. **Vision-Language Processing**: Combining computer vision with language understanding
4. **Voice-to-Action Pipeline**: Complete system for converting speech to robot actions
5. **ROS Actions**: Long-running tasks with feedback and cancellation
6. **Complete Integration**: End-to-end Vision-Language-Action system

The Vision-Language-Action integration represents the future of human-robot interaction, enabling robots to understand and respond to natural language commands while perceiving and acting in their environment. This integration is essential for creating robots that can work effectively alongside humans in natural settings.

The complete system demonstrates how modern AI technologies can be combined to create sophisticated robotic systems capable of complex interactions and tasks.