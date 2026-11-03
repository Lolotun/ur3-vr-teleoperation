using System;

// Commanded finger position, not measured position or gripping force.
// Independent of Unity input so button edges and latch behavior can be tested.
public sealed class GripperCommandState
{
    public float Position { get; private set; }
    public bool Latched { get; private set; }
    private bool primaryArmed;
    private bool primaryWasPressed;

    public void DisarmPrimary()
    {
        // Require an observed button release after tracking/calibration changes.
        primaryArmed = false;
        primaryWasPressed = false;
    }

    public void Update(float grip, bool primaryPressed)
    {
        if (float.IsNaN(grip) || float.IsInfinity(grip)) return;
        grip = Math.Max(0f, Math.Min(1f, grip));
        if (!primaryPressed) primaryArmed = true;
        bool toggle = primaryArmed && primaryPressed && !primaryWasPressed;
        primaryWasPressed = primaryPressed;

        if (toggle)
        {
            if (Latched)
            {
                Latched = false;
                Position = grip;
            }
            else
            {
                Position = grip;
                Latched = true;
            }
        }
        else if (!Latched)
        {
            Position = grip;
        }
    }
}
