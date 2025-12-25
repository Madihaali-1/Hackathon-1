---
sidebar_position: 4
---

# Interaction Patterns for Digital Twins

Designing effective interaction patterns is crucial for digital twin systems in humanoid robotics. This section explores various interaction paradigms and how to implement them in Unity for intuitive and efficient human-robot interaction.

## Interaction Pattern Categories

### 1. Direct Manipulation
Users directly interact with objects in the digital environment.

#### Implementation in Unity
```csharp
// Example: Direct robot control
using UnityEngine;

public class DirectRobotControl : MonoBehaviour
{
    [SerializeField] private Transform robotBody;
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float rotateSpeed = 100f;

    private Camera mainCamera;
    private bool isDragging = false;
    private Vector3 offset;

    void Start()
    {
        mainCamera = Camera.main;
    }

    void Update()
    {
        HandleMouseInput();
        HandleKeyboardInput();
    }

    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit))
            {
                if (hit.transform == robotBody)
                {
                    isDragging = true;
                    offset = robotBody.position - GetWorldPositionOnPlane(Input.mousePosition);
                }
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            isDragging = false;
        }

        if (isDragging)
        {
            robotBody.position = GetWorldPositionOnPlane(Input.mousePosition) + offset;
        }
    }

    void HandleKeyboardInput()
    {
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(horizontal, 0, vertical) * moveSpeed * Time.deltaTime;
        robotBody.Translate(movement);

        if (horizontal != 0)
        {
            robotBody.Rotate(Vector3.up, horizontal * rotateSpeed * Time.deltaTime);
        }
    }

    Vector3 GetWorldPositionOnPlane(Vector3 screenPosition)
    {
        Ray ray = mainCamera.ScreenPointToRay(screenPosition);
        Plane plane = new Plane(Vector3.up, Vector3.zero);

        if (plane.Raycast(ray, out float distance))
        {
            return ray.GetPoint(distance);
        }
        return Vector3.zero;
    }
}
```

### 2. Command-Based Interaction
Users issue commands through specialized interfaces.

#### Implementation in Unity
```csharp
// Example: Command interface
using UnityEngine;
using UnityEngine.UI;

public class RobotCommandInterface : MonoBehaviour
{
    [SerializeField] private InputField commandInput;
    [SerializeField] private Button executeButton;
    [SerializeField] private Text statusText;
    [SerializeField] private Transform robotTarget;

    void Start()
    {
        executeButton.onClick.AddListener(ExecuteCommand);
        commandInput.onEndEdit.AddListener(OnCommandSubmitted);
    }

    void OnCommandSubmitted(string command)
    {
        if (Input.GetKeyDown(KeyCode.Return))
        {
            ExecuteCommand();
        }
    }

    public void ExecuteCommand()
    {
        string command = commandInput.text.ToUpper();

        switch (command)
        {
            case "MOVE_FORWARD":
                MoveRobot(Vector3.forward);
                break;
            case "MOVE_BACKWARD":
                MoveRobot(Vector3.back);
                break;
            case "TURN_LEFT":
                RotateRobot(-90);
                break;
            case "TURN_RIGHT":
                RotateRobot(90);
                break;
            case "HOME":
                MoveToHomePosition();
                break;
            default:
                statusText.text = "Unknown command: " + command;
                return;
        }

        statusText.text = "Command executed: " + command;
        commandInput.text = "";
    }

    void MoveRobot(Vector3 direction)
    {
        robotTarget.Translate(direction * 1f, Space.World);
    }

    void RotateRobot(float angle)
    {
        robotTarget.Rotate(Vector3.up, angle);
    }

    void MoveToHomePosition()
    {
        robotTarget.position = Vector3.zero;
        robotTarget.rotation = Quaternion.identity;
    }
}
```

### 3. Gesture-Based Interaction
Users interact through gestures captured by input devices.

#### Implementation in Unity
```csharp
// Example: Gesture-based navigation
using UnityEngine;

public class GestureNavigation : MonoBehaviour
{
    [SerializeField] private Transform robot;
    [SerializeField] private float gestureThreshold = 0.1f;

    private Vector3 lastMousePosition;
    private bool gestureStarted = false;

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            lastMousePosition = Input.mousePosition;
            gestureStarted = true;
        }

        if (Input.GetMouseButtonUp(0) && gestureStarted)
        {
            Vector3 currentMousePosition = Input.mousePosition;
            Vector3 gestureVector = currentMousePosition - lastMousePosition;

            if (gestureVector.magnitude > gestureThreshold)
            {
                ProcessGesture(gestureVector);
            }
            gestureStarted = false;
        }
    }

    void ProcessGesture(Vector3 gestureVector)
    {
        // Determine gesture direction
        if (Mathf.Abs(gestureVector.x) > Mathf.Abs(gestureVector.y))
        {
            // Horizontal gesture
            if (gestureVector.x > 0)
                MoveRobotRight();
            else
                MoveRobotLeft();
        }
        else
        {
            // Vertical gesture
            if (gestureVector.y > 0)
                MoveRobotForward();
            else
                MoveRobotBackward();
        }
    }

    void MoveRobotForward() { robot.Translate(Vector3.forward * 0.5f); }
    void MoveRobotBackward() { robot.Translate(Vector3.back * 0.5f); }
    void MoveRobotLeft() { robot.Translate(Vector3.left * 0.5f); }
    void MoveRobotRight() { robot.Translate(Vector3.right * 0.5f); }
}
```

## Context-Aware Interactions

### Environment Context
Interactions that adapt based on the environment:

```csharp
// Example: Context-aware interaction
using UnityEngine;

public class ContextAwareInteraction : MonoBehaviour
{
    [SerializeField] private LayerMask interactionMask;
    [SerializeField] private float interactionDistance = 5f;

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, interactionDistance, interactionMask))
            {
                ProcessContextualInteraction(hit);
            }
        }
    }

    void ProcessContextualInteraction(RaycastHit hit)
    {
        string objectTag = hit.transform.tag;

        switch (objectTag)
        {
            case "Robot":
                HandleRobotInteraction(hit);
                break;
            case "Obstacle":
                HandleObstacleInteraction(hit);
                break;
            case "Waypoint":
                HandleWaypointInteraction(hit);
                break;
            default:
                Debug.Log("No interaction defined for: " + objectTag);
                break;
        }
    }

    void HandleRobotInteraction(RaycastHit hit)
    {
        Debug.Log("Interacting with robot: " + hit.transform.name);
        // Implement robot-specific interaction
    }

    void HandleObstacleInteraction(RaycastHit hit)
    {
        Debug.Log("Interacting with obstacle: " + hit.transform.name);
        // Implement obstacle-specific interaction
    }

    void HandleWaypointInteraction(RaycastHit hit)
    {
        Debug.Log("Interacting with waypoint: " + hit.transform.name);
        // Implement waypoint-specific interaction
    }
}
```

## Multi-Modal Interaction

### Combining Input Methods
Creating interfaces that combine multiple input modalities:

```csharp
// Example: Multi-modal interaction
using UnityEngine;

public class MultiModalInteraction : MonoBehaviour
{
    [SerializeField] private Transform robot;
    [SerializeField] private Camera mainCamera;

    void Update()
    {
        // Keyboard input
        HandleKeyboardInput();

        // Mouse input
        HandleMouseInput();

        // Voice command simulation (using keyboard shortcuts)
        HandleVoiceCommands();
    }

    void HandleKeyboardInput()
    {
        if (Input.GetKey(KeyCode.W))
            robot.Translate(Vector3.forward * Time.deltaTime);
        if (Input.GetKey(KeyCode.S))
            robot.Translate(Vector3.back * Time.deltaTime);
        if (Input.GetKey(KeyCode.A))
            robot.Translate(Vector3.left * Time.deltaTime);
        if (Input.GetKey(KeyCode.D))
            robot.Translate(Vector3.right * Time.deltaTime);
    }

    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit))
            {
                // Move robot to clicked position
                Vector3 targetPosition = new Vector3(hit.point.x, robot.position.y, hit.point.z);
                robot.position = targetPosition;
            }
        }
    }

    void HandleVoiceCommands()
    {
        // Simulating voice commands with number keys
        if (Input.GetKeyDown(KeyCode.Alpha1))
            ExecuteCommand("MOVE_TO_HOME");
        if (Input.GetKeyDown(KeyCode.Alpha2))
            ExecuteCommand("FOLLOW_PATH");
        if (Input.GetKeyDown(KeyCode.Alpha3))
            ExecuteCommand("STOP");
    }

    void ExecuteCommand(string command)
    {
        Debug.Log("Executing command: " + command);
        // Implement command execution logic
    }
}
```

## Interaction Feedback Systems

### Visual Feedback
Providing immediate visual feedback for user actions:

```csharp
// Example: Visual feedback system
using UnityEngine;

public class InteractionFeedback : MonoBehaviour
{
    [SerializeField] private Color highlightColor = Color.yellow;
    [SerializeField] private float highlightDuration = 0.2f;

    private Material originalMaterial;
    private Renderer targetRenderer;

    public void HighlightObject(GameObject target)
    {
        targetRenderer = target.GetComponent<Renderer>();
        if (targetRenderer != null)
        {
            originalMaterial = targetRenderer.material;
            targetRenderer.material.color = highlightColor;

            Invoke("ResetHighlight", highlightDuration);
        }
    }

    void ResetHighlight()
    {
        if (targetRenderer != null && originalMaterial != null)
        {
            targetRenderer.material = originalMaterial;
        }
    }

    public void ShowSelectionIndicator(Vector3 position)
    {
        GameObject indicator = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        indicator.transform.position = position;
        indicator.transform.localScale = Vector3.one * 0.1f;
        indicator.GetComponent<Renderer>().material.color = Color.green;

        Destroy(indicator, 1.0f);
    }
}
```

## Common Interaction Patterns

### 1. Point-and-Click Navigation
Users click on locations to move the robot:

- **Implementation**: Raycasting to determine target location
- **Feedback**: Visual indicator showing target position
- **Validation**: Check for obstacles or invalid positions

### 2. Waypoint-Based Navigation
Users set multiple waypoints for the robot to follow:

- **Implementation**: Store and execute waypoint sequences
- **Feedback**: Path visualization and progress indicators
- **Validation**: Check path feasibility and obstacle avoidance

### 3. Teleoperation
Direct control of robot movements:

- **Implementation**: Real-time command transmission
- **Feedback**: Immediate visual response to commands
- **Validation**: Safety limits and collision avoidance

## Exercise: Implement Multi-Modal Interface

1. Create a Unity scene with a robot model
2. Implement direct manipulation (click-to-move)
3. Add command-based interface (text input)
4. Include gesture-based navigation (keyboard shortcuts)
5. Add visual feedback for all interactions
6. Test the interface with different users and gather feedback

## Best Practices for Interaction Design

### 1. Consistency
- Use consistent interaction patterns throughout the interface
- Maintain visual and behavioral consistency
- Follow platform conventions where applicable

### 2. Discoverability
- Make interaction options clear and visible
- Provide tooltips and guidance for new users
- Use visual cues to indicate interactive elements

### 3. Responsiveness
- Provide immediate feedback for user actions
- Maintain consistent frame rates for smooth interaction
- Handle errors gracefully with clear messaging

### 4. Accessibility
- Support multiple interaction methods for different user preferences
- Provide keyboard alternatives for mouse-based interactions
- Consider users with different abilities and limitations

## Summary

Effective interaction patterns are essential for digital twin systems in humanoid robotics. By implementing direct manipulation, command-based interfaces, and gesture-based controls, you can create intuitive and efficient interfaces for human-robot interaction. Following best practices for consistency, discoverability, and accessibility ensures that your digital twin interfaces are usable by a wide range of users.