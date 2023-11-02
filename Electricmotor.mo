package Electricmotor
  model Gleichstrommotor
    //Parameter
    parameter Real J = 0.003 "Trägheitsmoment des Motors (kg*m^2)";
    parameter Real B = 0.5 "Viskoser Dämpfungskoeffizient (N*m*s/rad)";
    parameter Real K = 0.5 "Motor-Konstante (N*m/A)";
    parameter Real R = 1.0 "Wicklungswiderstand (Ohm)";
    parameter Real L = 0.1 "Wicklungsinduktivität (H)";
    parameter Real k_p = 0.5 "Proportionaler Regelungsfaktor";
    parameter Real k_i = 4 "Integraler Regelungsfaktor";
    parameter Real desiredSpeed = 20000 * 2 * 3.14159 / 60 "Gewünschte Drehzahl (rad/s)";
    parameter Real maxCurrent = 3500.0 "Maximal zulässiger Strom (A)";
    //Variablen
    output Real u "Stromzufuhr (A)";
    //input Real desiredSpeed "Gewünschte Drehzahl (rad/s)";
    output Real omega "Aktuelle Drehzahl (rad/s)";
    Real i (fixed = false) "Strom durch die Wicklung (A)";
    Real tau "Motor-Drehmoment (N*m)";
    Real integral_error "Integralfehler (rad)";
  equation
    tau = K*i;
    J*der(omega) = tau - B*omega;
    L*der(i) = min(maxCurrent, u) - R*i - K*omega;
// Regelung auf die gewünschte Drehzahl
    u = k_p*(desiredSpeed - omega) + k_i*integral_error;
// Berechnung des Integralfehlers
    der(integral_error) = desiredSpeed - omega;
  initial equation
    omega = 0;
    integral_error = 0;
    i = 0;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
  end Gleichstrommotor;
end Electricmotor;
