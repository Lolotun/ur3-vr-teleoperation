using System;

static class Program
{
    static void Check(bool condition, string message)
    {
        if (!condition) throw new Exception(message);
    }

    static void Near(float actual, float expected, string message)
    {
        Check(Math.Abs(actual - expected) < 0.00001f, message);
    }

    static void Main()
    {
        var state = new GripperCommandState();
        state.Update(0.25f, false);
        Near(state.Position, 0.25f, "Analog quarter grip");
        state.Update(0.6f, true);
        Check(state.Latched, "A latches");
        state.Update(0f, true);
        Near(state.Position, 0.6f, "Holding A must not toggle repeatedly");
        state.Update(0f, false);
        Near(state.Position, 0.6f, "Release Grip while latched holds position");
        state.Update(0f, true);
        Check(!state.Latched, "Second A resumes analog control");
        Near(state.Position, 0f, "Unlock with released Grip opens");
        state.Update(0.4f, false);
        state.Update(0.4f, true);
        state.Update(0.8f, false);
        state.Update(0.8f, true);
        Check(!state.Latched, "Unlock with pressed Grip resumes analog");
        Near(state.Position, 0.8f, "Unlock follows current Grip");

        state.Update(0.5f, false);
        state.Update(0.5f, true);
        state.DisarmPrimary();
        state.Update(0f, true);
        Check(state.Latched, "Tracking/calibration resume with A held must not unlock");
        Near(state.Position, 0.5f, "Tracking loss preserves latched position");
        state.Update(0f, false);
        state.Update(0f, true);
        Check(!state.Latched, "Release then A works after tracking resume");

        var startup = new GripperCommandState();
        startup.Update(0.3f, true);
        Check(!startup.Latched, "A held on startup is not an edge");
        startup.Update(0.3f, false);
        startup.Update(0.3f, true);
        Check(startup.Latched, "A works after an observed release");
        startup.Update(float.NaN, false);
        startup.Update(float.PositiveInfinity, false);
        Near(startup.Position, 0.3f, "Invalid input preserves command");

        var endpoints = new GripperCommandState();
        endpoints.Update(-1f, false);
        Near(endpoints.Position, 0f, "Negative input clamps");
        endpoints.Update(2f, false);
        Near(endpoints.Position, 1f, "Large input clamps");
        Console.WriteLine("Gripper analog/latch checks passed.");
    }
}
