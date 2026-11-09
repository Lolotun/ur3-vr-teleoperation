// Require release after tracking/connection loss; emit once per physical press.
public sealed class ButtonPressEdge
{
    private bool armed;
    private bool wasPressed;

    public bool Update(bool available, bool pressed)
    {
        if (!available)
        {
            armed = false;
            wasPressed = false;
            return false;
        }
        if (!pressed) armed = true;
        bool edge = armed && pressed && !wasPressed;
        wasPressed = pressed;
        return edge;
    }
}
