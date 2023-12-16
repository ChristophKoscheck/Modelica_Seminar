package Quadrocopter
  class Gehause
    // Konstanten
    import Modelica.Constants.g_n;
    // Parameter
    parameter Modelica.Units.SI.Mass masse = 87 "Masse Jetson One in kg";
    parameter Modelica.Units.SI.Mass masse_pilot = 95 "Masse Pilot in kg";
    parameter Modelica.Units.SI.Length abstand_rotoren = 1.8 "Abstand in m";
    parameter Modelica.Units.SI.Area A = 1.5 "Querschnittsfläche in m^2 für den Luftwiderstand";
    parameter Real c_w = 0.2 "Luftwiderstandsbeiwert für den Quadrocopter";
    parameter Modelica.Units.SI.Length breite = 2.5 "Breite des Quadrocopters";
    parameter Modelica.Units.SI.Length laenge = 1.0 "Länge des Quadrocopters";
    // Variablen
    Modelica.Units.SI.Position h "Höhe in vertikaler Richtung (m)";
    Modelica.Units.SI.Velocity v_y "Geschwindigkeit in vertikaler Richtung (m/s)";
    Modelica.Units.SI.Acceleration a_y "Beschleunigung in vertikaler Richtung (m/s^2)";
    Modelica.Units.SI.Force F_y "Gesamtauftriebskraft in vertikaler Richtung (N)";
    Modelica.Units.SI.Force F_gewicht "Gewichtskraft (N)";
    Modelica.Units.SI.Position x "Position in horizontaler Richtung (m)";
    Modelica.Units.SI.Velocity v_x "Geschwindigkeit in horizontaler Richtung (m/s)";
    Modelica.Units.SI.Acceleration a_x "Beschleunigung in horizontaler Richtung (m/s^2)";
    Modelica.Units.SI.Force F_x "Kraft in horizontaler Richtung (N)";
    Modelica.Units.SI.Angle Phi "Nickwinkel (rad)";
    Modelica.Units.SI.AngularVelocity Phi_p "Nickgeschwindigkeit (rad/s)";
    Modelica.Units.SI.AngularAcceleration Phi_pp "Nickbeschleunigung (rad/s^2)";
    Modelica.Units.SI.Inertia Jx "Massenträgheitsmoment (kg*m^2)";
    Modelica.Units.SI.Force F_diff "Differenz der Auftriebskräfte zwischen vorderen und hinteren Rotoren (N)";
    Modelica.Units.SI.Force F_v "Auftriebskraft der vorderen Rotoren (N)";
    Modelica.Units.SI.Force F_h "Auftriebskraft der hinteren Rotoren (N)";
    // Connector
    Quadrocopter.Mechanisch mechanisch annotation(
      Placement(visible = true, transformation(origin = {5, 61}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Quadrocopter.Mechanisch mechanisch1 annotation(
      Placement(visible = true, transformation(origin = {-56, -6}, extent = {{-14, -14}, {14, 14}}, rotation = 0), iconTransformation(origin = {-60, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Quadrocopter.Mechanisch mechanisch2 annotation(
      Placement(visible = true, transformation(origin = {2, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {2, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Quadrocopter.Mechanisch mechanisch3 annotation(
      Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {-76, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-70, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealOutput u annotation(
      Placement(visible = true, transformation(origin = {-74, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-70, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealOutput x_ist annotation(
      Placement(visible = true, transformation(origin = {-42, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-36, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
// Connectoren
    mechanisch.s = mechanisch1.s;
    mechanisch1.s = mechanisch2.s;
    mechanisch2.s = mechanisch3.s;
    h = mechanisch.s;
    y = h;
    u = Phi;
    x_ist = x;
// vertikale Bewegung
    der(h) = v_y;
    der(v_y) = a_y;
    F_gewicht = (masse + masse_pilot)*g_n;
    (masse + masse_pilot)*a_y = F_y - F_gewicht;
    der(x) = v_x;
    der(v_x) = a_x;
    (masse + masse_pilot)*a_x = F_x - (0.5*c_w*A*(v_x^2));
// Nickbewegung
    der(Phi) = Phi_p;
    der(Phi_p) = Phi_pp;
    Phi_pp = ((abstand_rotoren/2)/Jx)*F_diff;
// Berechnung des Massenträgheitsmoment
    Jx = (masse + masse_pilot)/12*(breite^2 + laenge^2);
// Zerlegung der Auftriebskraft der Rotoren in eine vertikale Kraft und eine horizontale Kraft
    F_y = cos(Phi)*(mechanisch.F + mechanisch1.F + mechanisch2.F + mechanisch3.F);
    F_x = sin(Phi)*(mechanisch.F + mechanisch1.F + mechanisch2.F + mechanisch3.F);
// Gruppierung der Rotoren in "vordere Rotoren" und "hintere Motoren"
    F_v = mechanisch.F + mechanisch1.F;
    F_h = mechanisch2.F + mechanisch3.F;
    F_diff = F_v - F_h "zum Debuggen";
// Modellierung des Bodens
    when h < 0 then
      reinit(h, 0.015);
      reinit(v_y, 0.01);
    end when;
    annotation(
      Icon(graphics = {Bitmap(origin = {-7, 4}, extent = {{-79, -42}, {79, 42}}, fileName = "modelica://Quadrocopter/Bilder/JetsonOne_Seitenansicht-removebg-preview_rotoren_entfernt_klein-removebg-preview.png")}, coordinateSystem(extent = {{-80, -40}, {80, 40}})),
      Diagram(graphics = {Text(origin = {-60, 12}, extent = {{-8, 6}, {8, -6}}, textString = "vorne"), Text(origin = {2, 80}, extent = {{-8, 6}, {8, -6}}, textString = "vorne")}, coordinateSystem(extent = {{-80, -40}, {80, 40}})),
      Documentation(info = "<html><head></head><body>

<p style=\"text-align:left\"><span style=\"font-family:Arial,Helvetica,sans-serif\"><strong><span style=\"font-size:24.0pt\">Dokumentation f&uuml;r ein Quadrocopter</span></strong></span></p>

<p style=\"text-align:left\"><span style=\"font-family:Arial,Helvetica,sans-serif\"><strong><span style=\"font-size:18.0pt\">Beschreibung</span></strong></span></p>

<p><span style=\"font-family:Arial,Helvetica,sans-serif\">Dieses Modelica-Modell beschreibt die Flugdynamik eines Quadrocopters. Es simuliert die horizontale und vertikale Bewegung des Quadrocopters sowie die Nickbewegung des Quadrokopters.</span></p>

<table border=\"1\" style=\"width:600px\">
 <tbody>
  <tr>
  	<th>Parameter</th>
  	<th>Einheit</th>
  	<th style=\"width:369px\">Beschreibung</th>
  </tr>
  <tr>
  	<td style=\"text-align:center\">masse</td>
  	<td style=\"text-align:center\">kg</td>
  	<td style=\"text-align:center; width:369px\">Masse des Jetson One</td>
  </tr>
  <tr>
  	<td style=\"text-align:center\">masse<sub>pilot</sub></td>
  	<td style=\"text-align:center\">kg</td>
  	<td style=\"text-align:center; width:369px\">Masse des Piloten</td>
  </tr>
  <tr>
  	<td style=\"text-align:center\">abstand<sub>rotoren</sub></td>
  	<td style=\"text-align:center\">m</td>
  	<td style=\"text-align:center; width:369px\">Abstand zwischen den Rotoren</td>
  </tr>
  <tr>
  	<td style=\"text-align:center\">A</td>
  	<td style=\"text-align:center\">m&sup2;</td>
  	<td style=\"text-align:center; width:369px\">Querschnittsfl&auml;che f&uuml;r den Luftwiderstand</td>
  </tr>
  <tr>
  	<td style=\"text-align:center\">c<sub>w</sub></td>
  	<td style=\"text-align:center\">-</td>
  	<td style=\"text-align:center; width:369px\">Luftwiderstandsbeiwert f&uuml;r den Quadrocopter</td>
  </tr>
  <tr>
  	<td style=\"text-align:center\">laenge</td>
  	<td style=\"text-align:center\">m</td>
  	<td style=\"text-align:center; width:369px\">L&auml;nge des Quadrocopters</td>
  </tr>
  <tr>
  	<td style=\"text-align:center\">breite</td>
  	<td style=\"text-align:center\">m</td>
  	<td style=\"text-align:center; width:369px\">Breite des Quadrocopters</td>
  </tr>
 </tbody>
</table>

<p>&nbsp;</p>

<table border=\"1\" style=\"width:600px\">
 <tbody>
  <tr>
  	<th style=\"width:145px\">Variable</th>
  	<th style=\"width:63px\">Einheit</th>
  	<th style=\"width:370px\">Beschreibung</th>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">h</td>
  	<td style=\"text-align:center; width:63px\">m</td>
  	<td style=\"text-align:center; width:370px\">H&ouml;he in vertikaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">v<sub>y</sub></td>
  	<td style=\"text-align:center; width:63px\">m/s</td>
  	<td style=\"text-align:center; width:370px\">Geschwindigkeit in vertikaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">a<sub>y</sub></td>
  	<td style=\"text-align:center; width:63px\">m/s&sup2;</td>
  	<td style=\"text-align:center; width:370px\">Beschleunigung in vertikaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">F<sub>y</sub></td>
  	<td style=\"text-align:center; width:63px\">N</td>
  	<td style=\"text-align:center; width:370px\">Gesamtauftriebskraft in vertikaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">F<sub>Gewicht</sub></td>
  	<td style=\"text-align:center; width:63px\">N</td>
  	<td style=\"text-align:center; width:370px\">Gewichtskraft</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">x</td>
  	<td style=\"text-align:center; width:63px\">m</td>
  	<td style=\"text-align:center; width:370px\">Position in horizontaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">v<sub>x</sub></td>
  	<td style=\"text-align:center; width:63px\">m/s</td>
  	<td style=\"text-align:center; width:370px\">Geschwindigkeit in horizontaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">a<sub>x</sub></td>
  	<td style=\"text-align:center; width:63px\">m/s&sup2;</td>
  	<td style=\"text-align:center; width:370px\">Beschleunigung in horizontaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">F<sub>x</sub></td>
  	<td style=\"text-align:center; width:63px\">N</td>
  	<td style=\"text-align:center; width:370px\">Kraft in horizontaler Richtung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">Phi</td>
  	<td style=\"text-align:center; width:63px\">rad</td>
  	<td style=\"text-align:center; width:370px\">Nickwinkel</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">Phi_p</td>
  	<td style=\"text-align:center; width:63px\">rad/s</td>
  	<td style=\"text-align:center; width:370px\">Nickgeschwindigkeit</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">Phi_pp</td>
  	<td style=\"text-align:center; width:63px\">
  	<p>rad/s&sup2;</p>
  	</td>
  	<td style=\"text-align:center; width:370px\">Nickbeschleunigung</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">J<sub>x</sub></td>
  	<td style=\"text-align:center; width:63px\">kg&middot;m&sup2;</td>
  	<td style=\"text-align:center; width:370px\">Massentr&auml;gheitsmoment</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">F<sub>diff</sub></td>
  	<td style=\"text-align:center; width:63px\">N</td>
  	<td style=\"text-align:center; width:370px\">Differenz der Auftriebskr&auml;fte zwischen vorderen und hinteren Rotoren</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">F<sub>v</sub></td>
  	<td style=\"text-align:center; width:63px\">N</td>
  	<td style=\"text-align:center; width:370px\">Auftriebskraft der vorderen Rotoren</td>
  </tr>
  <tr>
  	<td style=\"text-align:center; width:145px\">F<sub>h</sub></td>
  	<td style=\"text-align:center; width:63px\">N</td>
  	<td style=\"text-align:center; width:370px\">Auftriebskraft der hinteren Rotoren</td>
  </tr>
 </tbody>
</table>

<p><span style=\"font-family:Arial,Helvetica,sans-serif\"><strong><span style=\"font-size:18.0pt\">Annahmen und Modellierung des Quadrocopters</span></strong></span></p>

<p>In diesem Modell wird der Quadrocopter mit Vereinfachungen und Annahmen modelliert.</p>

<ul>
 <li>2D Modellierung des Quadrocopter mit 3 Freiheitsgraden (3DOF)</li>
 <li>Erdbeschleunigung mit 9,81 m/s&sup2;</li>
 <li>Luftwiderstand in der Horizontalbewegung</li>
</ul>

<p style=\"text-align:left\"><span style=\"font-family:Arial,Helvetica,sans-serif\"><strong><span style=\"font-size:18.0pt\">Bodenmodellierung</span></strong></span></p>

<p style=\"text-align:left\"><span style=\"font-family:Arial,Helvetica,sans-serif\">Die Modellierung des Bodens ist durch Bedingungen f&uuml;r den Bodenkontakt definiert, um eine realistische Simulation zu gew&auml;hrleisten.</span></p>

<p style=\"text-align:left\"><span style=\"font-family:Arial,Helvetica,sans-serif\"><strong><span style=\"font-size:18.0pt\">Annotationen</span></strong></span></p>

<p style=\"text-align:left\"><span style=\"font-family:Arial,Helvetica,sans-serif\">Das Modell enth&auml;lt Annotationen f&uuml;r die grafische Darstellung des Icons und des Diagramms.</span></p>


</body></html>"));
  end Gehause;

  model Steigflug
    //Variablen
    Modelica.Units.SI.Energy E_gesamt "elektrische Energieentnahme alle Motoren";
    //Klassen
    //Gehause gehause(position(start = 0));
    //Modelica-Blöcke
    Quadrocopter.Gehause gehause annotation(
      Placement(visible = true, transformation(origin = {5, 5.5}, extent = {{-75, -37.5}, {75, 37.5}}, rotation = 0)));
    Quadrocopter.Motor motor annotation(
      Placement(visible = true, transformation(origin = {46, 73.5}, extent = {{-10, 12.5}, {10, -12.5}}, rotation = 180)));
    Quadrocopter.Motor motor1 annotation(
      Placement(visible = true, transformation(origin = {46.45, -56.7}, extent = {{10.2, -12.75}, {-10.2, 12.75}}, rotation = 0)));
    Quadrocopter.Rotor rotor2 annotation(
      Placement(visible = true, transformation(origin = {-76.6667, -46.7333}, extent = {{-43.3333, 17.3333}, {43.3333, 43.3333}}, rotation = 90)));
    Quadrocopter.Motor motor2 annotation(
      Placement(visible = true, transformation(origin = {-55, -58.15}, extent = {{-9.4, -11.75}, {9.4, 11.75}}, rotation = 0)));
    Quadrocopter.Rotor rotor3 annotation(
      Placement(visible = true, transformation(origin = {-128.667, 54}, extent = {{-40, 16}, {40, 40}}, rotation = -90)));
    Quadrocopter.Motor motor3 annotation(
      Placement(visible = true, transformation(origin = {-55.6, 72.8}, extent = {{-9.6, -12}, {9.6, 12}}, rotation = 0)));
    Quadrocopter.Steuerung steuerung(kp_h = 1.3) annotation(
      Placement(visible = true, transformation(origin = {16, 130}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ramp(duration = 1, height = 300, startTime = 5) annotation(
      Placement(visible = true, transformation(origin = {-82, 136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ramp1(duration = 1, height = 200, startTime = 60) annotation(
      Placement(visible = true, transformation(origin = {-46, 118}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Quadrocopter.Rotor rotor annotation(
      Placement(visible = true, transformation(origin = {103, 67}, extent = {{-41, 16.4}, {41, 41}}, rotation = -90)));
    Quadrocopter.Rotor rotor1 annotation(
      Placement(visible = true, transformation(origin = {140.333, -37.0667}, extent = {{-44.3333, 17.7333}, {44.3333, 44.3333}}, rotation = 90)));
  equation
//Berechnung der Entnahme von elektrischer Energie aller Motoren mit Inverter an Batterie
    E_gesamt = motor.E_einzel + motor1.E_einzel + motor2.E_einzel + motor3.E_einzel;
// Connector
    connect(ramp.y, steuerung.u) annotation(
      Line(points = {{-71, 136}, {-17, 136}, {-17, 134}, {9, 134}}, color = {0, 0, 127}));
    connect(steuerung.X, ramp1.y) annotation(
      Line(points = {{8.6, 126.2}, {-15.8, 126.2}, {-15.8, 118}, {-35, 118}}, color = {0, 0, 127}));
    connect(rotor3.mechanisch, gehause.mechanisch3) annotation(
      Line(points = {{-101, 49}, {61, 49}, {61, 5.5}}));
    connect(rotor.mechanisch, gehause.mechanisch) annotation(
      Line(points = {{132, 62}, {5, 62}}));
    connect(rotor1.mechanisch, gehause.mechanisch1) annotation(
      Line(points = {{109, -32}, {20, -32}, {20, 4}, {-51, 4}}));
    connect(motor2.m_rot, rotor2.m_rot) annotation(
      Line(points = {{-62, -57}, {-70, -57}, {-70, -47}, {-107, -47}}));
    connect(motor3.m_rot, rotor3.m_rot) annotation(
      Line(points = {{-62, 74}, {-117.5, 74}, {-117.5, 54}, {-101, 54}}));
    connect(motor.m_rot, rotor.m_rot) annotation(
      Line(points = {{54, 74}, {70, 74}, {70, 67}, {132, 67}}));
    connect(motor1.m_rot, rotor1.m_rot) annotation(
      Line(points = {{54, -55}, {72, -55}, {72, -37}, {109, -37}}));
    connect(gehause.x_ist, steuerung.x_ist) annotation(
      Line(points = {{-29, 71}, {84, 71}, {84, 8}, {98, 8}, {98, 112}, {12, 112}, {12, 122}}, color = {0, 0, 127}));
    connect(gehause.y, steuerung.h_ist) annotation(
      Line(points = {{-61, 45}, {-61, 48}, {16, 48}, {16, 122}}, color = {0, 0, 127}));
    connect(gehause.u, steuerung.Phi_ist) annotation(
      Line(points = {{-61, -28}, {20, -28}, {20, 122}}, color = {0, 0, 127}));
    connect(steuerung.omega_soll_vorne, motor.soll_v) annotation(
      Line(points = {{26, 134}, {46, 134}, {46, 82}}, color = {0, 0, 127}));
    connect(steuerung.omega_soll_vorne, motor1.soll_v) annotation(
      Line(points = {{26, 134}, {46, 134}, {46, -49}}, color = {0, 0, 127}));
    connect(steuerung.omega_soll_hinten, motor3.soll_v) annotation(
      Line(points = {{26, 126}, {34, 126}, {34, 96}, {-54, 96}, {-54, 80}}, color = {0, 0, 127}));
    connect(steuerung.omega_soll_hinten, motor2.soll_v) annotation(
      Line(points = {{26, 126}, {34, 126}, {34, 96}, {-54, 96}, {-54, -51}}, color = {0, 0, 127}));
    connect(gehause.mechanisch2, rotor2.mechanisch) annotation(
      Line(points = {{7, -51}, {7, -42}, {-107, -42}}));
    annotation(
      experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 1),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 150}})),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 150}}), graphics = {Bitmap(origin = {3, 6}, extent = {{-133, -56}, {133, 56}}, fileName = "modelica://Quadrocopter/Bilder/JetsonOneDummy-removebg-preview.png")}));
  end Steigflug;

  class Rotor
    // Constants
    import Modelica.Constants.pi;
    // Parameter
    parameter Modelica.Units.SI.Length r = 0.3 "mittlere Rotorblattlänge in m";
    //parameter Real n(unit="1/min") = 1000 "Drehzahl";
    parameter Real c_T = 0.4 "Auftriebsbeiwert";
    parameter Real c_W = 0.03 "Strömungswiderstandsbeiwert";
    parameter Real rho = 1.225 "Dichte von Luft Referenzwert";
    parameter Modelica.Units.SI.Area A = 0.5 "Fläche in m^2";
    //Variablen
    Modelica.Units.SI.AngularVelocity omega "gemittelte Rotorwinkelgeschwindigkeit in rad/s";
    Modelica.Units.SI.Force F_a "Auftriebskraft";
    // Connector
    Quadrocopter.Mechanisch mechanisch annotation(
      Placement(visible = true, transformation(origin = {0, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {12, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Quadrocopter.M_rot m_rot annotation(
      Placement(visible = true, transformation(origin = {40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {4.44089e-16, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  equation
    omega = der(m_rot.phi);
    m_rot.tau = r*0.5*c_W*rho*A*(r*omega)^2;
    F_a = c_T*rho*A*(r*omega)^2;
    0 = mechanisch.F + F_a;
    annotation(
      Icon(coordinateSystem(extent = {{-100, 40}, {100, 100}}), graphics = {Bitmap(origin = {5, 77}, extent = {{-110, 54}, {110, -54}}, fileName = "modelica://Quadrocopter/Bilder/Propeller2-removebg-preview.png")}),
      Diagram(coordinateSystem(extent = {{-100, 40}, {100, 100}})),
      experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
  end Rotor;

  connector Mechanisch
    // Variablen
    flow Modelica.Units.SI.Force F;
    Modelica.Units.SI.Position s "Weg";
    annotation(
      Icon(graphics = {Rectangle(fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Text(origin = {0, 2}, textColor = {255, 255, 255}, extent = {{-70, 68}, {70, -68}}, textString = "T")}, coordinateSystem(extent = {{-80, -80}, {80, 80}})),
      Diagram(coordinateSystem(extent = {{-80, -80}, {80, 80}})));
  end Mechanisch;

  model Motor
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.DcdcInverter dcdcInverter(Td = 0.5/dcpmDriveData.fS, Tmf = 2/dcpmDriveData.fS, VMax = dcpmDriveData.VBat, fS = dcpmDriveData.fS) annotation(
      Placement(visible = true, transformation(origin = {62, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.Battery battery(INominal = 1000, Ri(displayUnit = "mOhm") = 0.0001, V0 = 52) annotation(
      Placement(visible = true, transformation(origin = {62, 56}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(IaNominal = dcpmData.IaNominal, Jr(fixed = true, start = dcpmData.Jr), Js = dcpmData.Js, La(displayUnit = "mH") = 1, Ra(displayUnit = "mOhm") = 0.03000000000000001, TaNominal = dcpmData.TaNominal, TaRef = 293.15, VaNominal = dcpmData.VaNominal, alpha20a = 0, ia(fixed = true), phiMechanical(displayUnit = "rad", fixed = true), wMechanical(displayUnit = "rad/s", fixed = true), wNominal = dcpmData.wNominal) annotation(
      Placement(visible = true, transformation(origin = {62, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.DriveDataDCPM dcpmDriveData(VBat = 60, fS = 4e3, motorData = dcpmData) annotation(
      Placement(visible = true, transformation(origin = {120, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData dcpmData(IaNominal = 500, Jr = 10e-3, Js = 10e-3, La(displayUnit = "mH") = 0.001, Ra(displayUnit = "mOhm") = 0.03000000000000001, TaNominal = 293.15, TaRef = 293.15, VaNominal = 48, wNominal = 4500*2*Modelica.Constants.pi/60) annotation(
      Placement(visible = true, transformation(origin = {120, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Quadrocopter.M_rot m_rot annotation(
      Placement(visible = true, transformation(origin = {112, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-45, 7}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI currentPI(KFF = dcpmDriveData.kPhi, Ti = dcpmDriveData.TiI*50, initType = Modelica.Blocks.Types.Init.InitialOutput, k = dcpmDriveData.kpI, symmetricLimits = false, useFF = true) annotation(
      Placement(visible = true, transformation(origin = {24, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
      Placement(visible = true, transformation(origin = {92, -44}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI speedPI(Ti = dcpmDriveData.Tiw, k = dcpmDriveData.kpw, symmetricLimits = false, yMax = dcpmDriveData.tauMax, yMin = 0) annotation(
      Placement(visible = true, transformation(origin = {-50, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain tau2i(k = 1/dcpmDriveData.kPhi) annotation(
      Placement(visible = true, transformation(origin = {-12, 16}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput soll_v annotation(
      Placement(visible = true, transformation(origin = {-144, 16}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {4, 38}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    //Variablen
    Modelica.Units.SI.Energy E_einzel "elektrische Energieentnahme Batterie";
    Modelica.Units.SI.Power P_einzel "elektrische Leistung";
  equation
    connect(battery.pin_n, dcdcInverter.pin_nBat) annotation(
      Line(points = {{56, 46}, {56, 26}}, color = {0, 0, 255}));
    connect(battery.pin_p, dcdcInverter.pin_pBat) annotation(
      Line(points = {{68, 46}, {68, 26}}, color = {0, 0, 255}));
    connect(dcdcInverter.pin_pMot, dcpm.pin_ap) annotation(
      Line(points = {{68, 6}, {68, -10}}, color = {0, 0, 255}));
    connect(dcdcInverter.pin_nMot, dcpm.pin_an) annotation(
      Line(points = {{56, 6}, {56, -10}}, color = {0, 0, 255}));
    connect(dcpm.flange, m_rot) annotation(
      Line(points = {{72, -20}, {112, -20}}));
    connect(currentPI.yMaxVar, dcdcInverter.vDC) annotation(
      Line(points = {{36, 22}, {52, 22}}, color = {0, 0, 127}));
    connect(currentPI.y, dcdcInverter.vRef) annotation(
      Line(points = {{35, 16}, {49, 16}}, color = {0, 0, 127}));
    connect(dcdcInverter.iMot, currentPI.u_m) annotation(
      Line(points = {{51, 10}, {43, 10}, {43, -4}, {17, -4}, {17, 4}}, color = {0, 0, 127}));
    connect(speedSensor.flange, dcpm.flange) annotation(
      Line(points = {{92, -34}, {92, -20}, {72, -20}}));
    connect(speedSensor.w, currentPI.feedForward) annotation(
      Line(points = {{92, -55}, {24, -55}, {24, 3}}, color = {0, 0, 127}));
    connect(speedSensor.w, speedPI.u_m) annotation(
      Line(points = {{92, -55}, {-56, -55}, {-56, 3}}, color = {0, 0, 127}));
    connect(speedPI.y, tau2i.u) annotation(
      Line(points = {{-38, 16}, {-24, 16}}, color = {0, 0, 127}));
    connect(tau2i.y, currentPI.u) annotation(
      Line(points = {{0, 16}, {12, 16}}, color = {0, 0, 127}));
    connect(speedPI.u, soll_v) annotation(
      Line(points = {{-62, 16}, {-144, 16}}, color = {0, 0, 127}));
//Berechnung der Entnahme von elektrischer Energie eines Motors mit Inverter
    P_einzel = battery.pin_n.i*battery.pin_p.v;
    der(E_einzel) = P_einzel;
    annotation(
      Icon(graphics = {Bitmap(origin = {0, -1}, rotation = 180, extent = {{70, 59}, {-70, -59}}, imageSource = "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")}, coordinateSystem(extent = {{-60, -60}, {60, 60}})),
      Diagram(coordinateSystem(extent = {{-60, -60}, {60, 60}})),
      experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.1));
  end Motor;

  connector M_rot
    Modelica.Units.SI.Angle phi "Absolute rotation angle of flange";
    flow Modelica.Units.SI.Torque tau "Cut torque in the flange";
    annotation(
      Icon(graphics = {Ellipse(fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Text(origin = {0, 4}, textColor = {255, 255, 255}, extent = {{-48, 50}, {48, -50}}, textString = "R")}, coordinateSystem(extent = {{-80, -80}, {80, 80}})),
      Diagram(coordinateSystem(extent = {{-80, -80}, {80, 80}})));
  end M_rot;

  model Steuerung
    parameter Real kp_h = 1.2 "Proportionalwert für Höhenregler";
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-100, 64}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-72, 42}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput omega_soll_vorne annotation(
      Placement(visible = true, transformation(origin = {100, 64}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {96, 44}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI heightController(Ti = 50, initType = Modelica.Blocks.Types.Init.InitialOutput, k = kp_h, symmetricLimits = false, useI = true, yMax = 400, yMin = 0) "PI-Regler" annotation(
      Placement(visible = true, transformation(origin = {2, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput h_ist "Ist-Höhe in m" annotation(
      Placement(visible = true, transformation(origin = {-4, 34}, extent = {{-8, -8}, {8, 8}}, rotation = 90), iconTransformation(origin = {-6, -76}, extent = {{-14, -14}, {14, 14}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput X annotation(
      Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-74, -38}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI Winkelregler(Ti = 18, initType = Modelica.Blocks.Types.Init.InitialOutput, k = 7.4, symmetricLimits = true, useI = true, yMax = 1) "PI-Regler für Nickwinkel Phi" annotation(
      Placement(visible = true, transformation(origin = {10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput Phi_ist annotation(
      Placement(visible = true, transformation(origin = {4, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {30, -76}, extent = {{-14, -14}, {14, 14}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealOutput omega_soll_hinten annotation(
      Placement(visible = true, transformation(origin = {100, -52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {97, -39}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
    Modelica.Blocks.Math.Add add(k2 = -1) annotation(
      Placement(visible = true, transformation(origin = {58, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput x_ist annotation(
      Placement(visible = true, transformation(origin = {-36, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-42, -76}, extent = {{-14, -14}, {14, 14}}, rotation = 90)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI XRegler(Ti = 1, initType = Modelica.Blocks.Types.Init.InitialOutput, k = 0.006, symmetricLimits = false, useI = false, yMax = 0.5, yMin = -0.5) "P-Regler für die horizontale Bewegung" annotation(
      Placement(visible = true, transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter_x(Falling = -4, Rising = 4) annotation(
      Placement(visible = true, transformation(origin = {-62, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter_h(Falling = -15, Rising = 15) annotation(
      Placement(visible = true, transformation(origin = {-54, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(h_ist, heightController.u_m) annotation(
      Line(points = {{-4, 34}, {-4, 52}}, color = {0, 0, 127}));
    connect(heightController.y, omega_soll_vorne) annotation(
      Line(points = {{13, 64}, {99, 64}}, color = {0, 0, 127}));
    connect(Phi_ist, Winkelregler.u_m) annotation(
      Line(points = {{4, -96}, {4, -72}}, color = {0, 0, 127}));
    connect(heightController.y, add.u1) annotation(
      Line(points = {{14, 64}, {28, 64}, {28, -46}, {46, -46}}, color = {0, 0, 127}));
    connect(add.y, omega_soll_hinten) annotation(
      Line(points = {{69, -52}, {100, -52}}, color = {0, 0, 127}));
    connect(Winkelregler.y, add.u2) annotation(
      Line(points = {{21, -60}, {34.5, -60}, {34.5, -58}, {46, -58}}, color = {0, 0, 127}));
    connect(XRegler.y, Winkelregler.u) annotation(
      Line(points = {{-19, -60}, {-2, -60}}, color = {0, 0, 127}));
    connect(XRegler.u_m, x_ist) annotation(
      Line(points = {{-36, -72}, {-36, -96}}, color = {0, 0, 127}));
    connect(X, slewRateLimiter_x.u) annotation(
      Line(points = {{-100, -60}, {-74, -60}}, color = {0, 0, 127}));
    connect(slewRateLimiter_x.y, XRegler.u) annotation(
      Line(points = {{-50, -60}, {-42, -60}}, color = {0, 0, 127}));
    connect(u, slewRateLimiter_h.u) annotation(
      Line(points = {{-100, 64}, {-66, 64}}, color = {0, 0, 127}));
    connect(slewRateLimiter_h.y, heightController.u) annotation(
      Line(points = {{-42, 64}, {-10, 64}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {85, 255, 127}, fillPattern = FillPattern.Horizontal, extent = {{-80, 80}, {80, -80}}), Text(origin = {0, 4}, extent = {{-48, 52}, {48, -52}}, textString = "S")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end Steuerung;

  class Battery
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {100, 18}, extent = {{-70, 110}, {-50, 90}}, rotation = 90), iconTransformation(origin = {0, -98}, extent = {{-70, 110}, {-50, 90}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {-42, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
      Placement(visible = true, transformation(origin = {100, -28}, extent = {{50, 110}, {70, 90}}, rotation = 90), iconTransformation(origin = {0, -98}, extent = {{50, 110}, {70, 90}}, rotation = 0)));
    parameter Modelica.Electrical.Batteries.ParameterRecords.CellData cellData1(OCVmax = 4.2, OCVmin = 2.5, Qnom = 18000, Ri = cellData1.OCVmax/1200) annotation(
      Placement(visible = true, transformation(origin = {-10, -28}, extent = {{60, 20}, {80, 40}}, rotation = 0)));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack battery1(Np = 3, Ns = 13, SOC(fixed = true, start = 0.95), cellData = cellData1, useHeatPort = false) annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
  equation
    connect(ground.p, pin_n) annotation(
      Line(points = {{-42, -62}, {-42, -24}, {0, -24}, {0, -42}}, color = {0, 0, 255}));
    connect(battery1.p, pin_p) annotation(
      Line(points = {{0, 10}, {0, 32}}, color = {0, 0, 255}));
    connect(battery1.n, pin_n) annotation(
      Line(points = {{0, -10}, {0, -42}}, color = {0, 0, 255}));
    annotation(
      Diagram,
      Icon(graphics = {Rectangle(origin = {0, 3}, extent = {{-50, 33}, {50, -33}})}));
  end Battery;
  annotation(
    uses(Modelica(version = "4.0.0")));
end Quadrocopter;
