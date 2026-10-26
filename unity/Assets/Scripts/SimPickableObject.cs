using UnityEngine;

/// <summary>
/// Marks an object as pickable by SimGripperPickup in simulation.
/// </summary>
public class SimPickableObject : MonoBehaviour
{
    [Tooltip("Optional override for the attach point. Defaults to this object's transform.")]
    public Transform attachPoint;
}
