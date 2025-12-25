---
sidebar_position: 3
title: "Unity Simulation for Robotics"
---

# Module 2: Simulation - Gazebo & Unity
## Chapter 3: Unity Simulation for Robotics

### Overview
Unity is a powerful game engine that has found significant applications in robotics simulation. With its high-quality graphics rendering, physics engine, and VR/AR capabilities, Unity provides an alternative approach to robotics simulation that emphasizes visual fidelity and immersive experiences.

### Unity in Robotics Context
Unity brings several unique advantages to robotics simulation:

**High-Quality Graphics**: Photorealistic rendering for computer vision training
**VR/AR Support**: Immersive interfaces for robot teleoperation and training
**Physics Engine**: PhysX for realistic physics simulation
**Asset Store**: Extensive library of 3D models and environments
**Cross-Platform**: Deploy to multiple platforms including mobile and VR

### Setting Up Unity for Robotics
To use Unity for robotics, you'll need several components:

**Unity Installation:**
1. Download Unity Hub from Unity's website
2. Install Unity Editor (2021.3 LTS or newer recommended)
3. Install required packages through Unity Package Manager

**Robotics Packages:**
- Unity Robotics Package: Core ROS/ROS 2 integration
- Unity Perception Package: Tools for generating synthetic data
- ML-Agents: Machine learning framework for training agents
- XR packages: For VR/AR applications

### Unity Robotics Package
The Unity Robotics Package provides the bridge between Unity and ROS/ROS 2:

**Installation:**
1. Open Unity Package Manager (Window > Package Manager)
2. Install "ROS TCP Connector" package
3. Install "Unity Robotics Package" 
4. Configure ROS/ROS 2 connection settings

**Basic Connection Setup:**
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs.StringMsg>("chatter");
    }
    
    void Update()
    {
        // Send a message every second
        if (Time.time % 1.0f < Time.deltaTime)
        {
            var msg = new Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs.StringMsg("Hello from Unity!");
            ros.Publish("chatter", msg);
        }
    }
}
```

### Creating Robot Models in Unity
Unity uses GameObjects to represent robots and their components:

**Robot Structure:**
```
Robot (GameObject)
├── Base (MeshRenderer + Collider)
├── Joint1 (HingeJoint + ConfigurableJoint)
├── Link1 (MeshRenderer + Collider)
├── Joint2 (HingeJoint + ConfigurableJoint)
├── Link2 (MeshRenderer + Collider)
└── Sensor (Custom Component)
```

**Example Robot Arm:**
```csharp
using UnityEngine;

public class RobotArm : MonoBehaviour
{
    public Transform[] joints;
    public float[] jointAngles;
    public float[] jointLimitsMin;
    public float[] jointLimitsMax;
    
    void Start()
    {
        jointAngles = new float[joints.Length];
        jointLimitsMin = new float[] {-90, -45, -90, -45, -90, -45};
        jointLimitsMax = new float[] {90, 45, 90, 45, 90, 45};
    }
    
    public void SetJointAngles(float[] angles)
    {
        for (int i = 0; i < joints.Length && i < angles.Length; i++)
        {
            // Apply joint limits
            float clampedAngle = Mathf.Clamp(angles[i], jointLimitsMin[i], jointLimitsMax[i]);
            jointAngles[i] = clampedAngle;
            
            // Apply rotation
            joints[i].localRotation = Quaternion.Euler(0, clampedAngle, 0);
        }
    }
}
```

### Sensor Simulation in Unity
Unity can simulate various types of sensors:

**Camera Sensor:**
```csharp
using UnityEngine;

public class CameraSensor : MonoBehaviour
{
    public Camera cam;
    public string topicName = "/camera/rgb/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    
    void Start()
    {
        cam = GetComponent<Camera>();
        cam.aspect = (float)imageWidth / imageHeight;
        cam.orthographic = false; // Use perspective camera
    }
    
    void Update()
    {
        // Capture image and send to ROS
        Texture2D image = CaptureImage();
        // Send image to ROS topic
    }
    
    Texture2D CaptureImage()
    {
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        
        cam.Render();
        
        Texture2D image = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        image.Apply();
        
        RenderTexture.active = currentRT;
        return image;
    }
}
```

**Lidar Sensor Simulation:**
```csharp
using UnityEngine;
using System.Collections.Generic;

public class LidarSensor : MonoBehaviour
{
    public int rayCount = 360;
    public float maxDistance = 10.0f;
    public float fieldOfView = 360.0f;
    
    void Update()
    {
        List<float> ranges = new List<float>();
        
        for (int i = 0; i < rayCount; i++)
        {
            float angle = (i * fieldOfView / rayCount) * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            direction = transform.TransformDirection(direction);
            
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxDistance))
            {
                ranges.Add(hit.distance);
            }
            else
            {
                ranges.Add(maxDistance);
            }
        }
        
        // Publish ranges to ROS topic
        PublishLidarData(ranges);
    }
    
    void PublishLidarData(List<float> ranges)
    {
        // Send ranges to ROS topic
    }
}
```

### Unity Perception Package
The Unity Perception Package enables generation of synthetic training data:

**Key Features:**
- Synthetic image generation with annotations
- Ground truth data extraction
- Sensor simulation
- Domain randomization

**Example Perception Configuration:**
```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;

public class PerceptionSetup : MonoBehaviour
{
    void Start()
    {
        // Add perception camera
        var perceptionCamera = GetComponent<PerceptionCamera>();
        if (perceptionCamera == null)
        {
            perceptionCamera = gameObject.AddComponent<PerceptionCamera>();
        }
        
        // Configure camera settings
        perceptionCamera.captureRgbImages = true;
        perceptionCamera.rgbCaptureSettings.outputWidth = 640;
        perceptionCamera.rgbCaptureSettings.outputHeight = 480;
        
        // Enable semantic segmentation
        var semanticSegmentationLabeler = GetComponent<SemanticSegmentationLabeler>();
        if (semanticSegmentationLabeler == null)
        {
            semanticSegmentationLabeler = gameObject.AddComponent<SemanticSegmentationLabeler>();
        }
    }
}
```

### ML-Agents Integration
Unity's ML-Agents framework allows training AI agents in simulation:

**Installation:**
```bash
pip install mlagents
```

**Creating a Learning Environment:**
```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotAgent : Agent
{
    public override void OnEpisodeBegin()
    {
        // Reset environment at start of episode
        transform.position = new Vector3(0, 0, 0);
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        // Add observations to sensor
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.rotation);
    }
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Process actions from neural network
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];
        
        transform.position += new Vector3(moveX, 0, moveZ) * Time.deltaTime;
        
        // Set reward based on task
        SetReward(CalculateReward());
    }
    
    float CalculateReward()
    {
        // Calculate reward based on task completion
        return 0.0f;
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
```

### Physics Simulation in Unity
Unity uses the PhysX physics engine for realistic simulation:

**Physics Settings:**
```csharp
using UnityEngine;

public class PhysicsSetup : MonoBehaviour
{
    void Start()
    {
        // Configure physics settings
        Physics.gravity = new Vector3(0, -9.81f, 0);
        Physics.defaultSolverIterations = 8;
        Physics.defaultSolverVelocityIterations = 1;
        Physics.sleepThreshold = 0.005f;
    }
}
```

**Rigidbody Configuration for Robot Links:**
```csharp
using UnityEngine;

public class RobotLink : MonoBehaviour
{
    public float mass = 1.0f;
    public bool isKinematic = false;
    
    void Start()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
        
        rb.mass = mass;
        rb.isKinematic = isKinematic;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }
}
```

### Best Practices for Unity Robotics Simulation
1. **Performance Optimization**: Use appropriate Level of Detail (LOD) for complex scenes
2. **Physics Tuning**: Adjust physics parameters to match real-world behavior
3. **Lighting Consistency**: Use consistent lighting for computer vision training
4. **Domain Randomization**: Vary environmental parameters to improve model robustness
5. **Validation**: Compare simulation results with real-world data when possible

### Exercises and Questions

1. What are the main advantages of using Unity for robotics simulation?
2. How do you set up the Unity Robotics Package for ROS/ROS 2 communication?
3. Explain the structure of a robot model in Unity.
4. How do you simulate camera and lidar sensors in Unity?
5. What is the Unity Perception Package and how is it used?
6. Describe the process of creating a learning environment with ML-Agents.
7. How do you configure physics settings in Unity for robotics simulation?
8. What are some best practices for optimizing Unity simulation performance?
9. Explain how domain randomization improves model robustness.
10. Compare the use cases for Unity vs Gazebo in robotics simulation.