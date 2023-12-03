package Motor
   
model BLDCMotorWithFriction
    parameter Real R = 2.0 "Wicklungswiderstand (Ohm)";
    parameter Real L = 0.01 "Wicklungsinduktivität (H)";
    parameter Real J = 0.1 "Trägheitsmoment des Rotors (kg*m^2)";
    parameter Real b = 0.01 "Reibungskoeffizient (N*m*s/rad)";
    parameter Real ke = 0.01 "Elektromotorische Kraft Konstante (V/rad/s)";
    parameter Real kv = 10 "Drehmoment-Konstante (N*m/A)";
    parameter Real w_ref = 300 "Nenndrehzahl (rad/s)";
    parameter Real V_nominal = 12.0 "Nennspannung (V)";
    parameter Real Kp = 0.1 "Proportional gain";
    parameter Real Ki = 0.01 "Integral gain";
    output Real w "Rotationsgeschwindigkeit (rad/s)";
    Real i "Phasenstrom (A)";
    Real tau "Drehmoment (N*m)";
    Real e "Regelfehler (rad/s)";
    Real u "Reglerausgang (V)";
  equation
    tau = kv*i - b*w;
    J*der(w) = tau - b*w;
    L*der(i) = u - R*i - ke*w;
    e = w_ref - w;
    u = min(V_nominal, max(0, Kp*e + Ki*time*e));
  initial equation
    w = 0;
    i = 0;
    annotation(
      experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 2),
  Diagram(graphics = {Rectangle(origin = {1, 1}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-43, 47}, {43, -47}})}));
  end BLDCMotorWithFriction;

  model Batterie
    parameter Real capacity = 10 "Kapazität der Batterie (Ah)";
    parameter Real V_nominal = 12.0 "Nennspannung der Batterie (V)";
    Real SOC "Ladezustand der Batterie";
    Real i "Batteriestrom (A)";
    Real V "Batteriespannung (V)";
  equation
    der(SOC) = -i / capacity;
    V = V_nominal * SOC;
    assert(SOC >= 0, "Batterie ist leer");
  initial equation
    SOC = 1; // Batterie ist zu Beginn voll geladen
  annotation(
      Diagram(graphics = {Rectangle(lineColor = {85, 0, 255}, fillColor = {85, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-40, 46}, {40, -46}})}));
end Batterie;

  connector MotorConnector
    flow Real tau "Drehmoment (N*m)";
    Real w "Rotationsgeschwindigkeit (rad/s)";annotation(
      Diagram(graphics = {Rectangle(origin = {0, -1}, lineColor = {0, 170, 0}, fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-28, 31}, {28, -31}})}));
  end MotorConnector;

  connector BatteryConnector
    flow Real i "Strom (A)";
    Real v "Spannung (V)";annotation(
      Diagram(graphics = {Ellipse(origin = {1, -1}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-27, 25}, {27, -25}})}));
  end BatteryConnector;
end Motor;
