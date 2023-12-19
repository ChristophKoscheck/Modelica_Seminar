package Drone
  package UserGuide
    extends Modelica.Icons.Information;

    class Contact "Contact"
      extends Modelica.Icons.Contact;
      annotation(
        Documentation(info = "<html><head></head><body><dl><dt><!--StartFragment--><p><font size=\"4\">The <b><font color=\"#2c3e50\">Modelica Drone Library</font></b> (this Modelica package) is developed by master students from Hochschule Pforzheim in their system engineering program:</font></p>
  <ul>
  <li><font size=\"4\"><b><font color=\"#2c3e50\">Christoph Koscheck</font></b> [koscheck@hs-pforzheim.de]</font></li>
  <li><font size=\"4\"><b><font color=\"#2c3e50\">Alexander Leitz</font></b> [leitzale@hs-pforzheim.de]</font></li>
  <li><font size=\"4\"><b><font color=\"#2c3e50\">Paul Smidt</font></b> [smidtpau@hs-pforzheim.de]</font></li>
  </ul><div><font size=\"4\">This project is also available on <b><font color=\"#2c3e50\">GitHub</font></b>:</font></div><div><a href=\"https://github.com/ChristophKoscheck/Modelica_Seminar\"><font size=\"4\">https://github.com/ChristophKoscheck/Modelica_Seminar</font></a></div><div><br></div>
  
  </dt></dl></body></html>"));
    end Contact;
    annotation(
      Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">This is a user guide to help the user understand and use the \"Drone\" library.</font></p><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">It also includes the contact information of the creators.</font></p>

</body></html>"));
  end UserGuide;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model DefDroneSim
      // Model Definitions --------------------
      // Required simulation models --------------------
      Drone.Environment.DefHeightProfile defHeightProfile annotation(
        Placement(visible = true, transformation(origin = {-3, 81}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
      Drone.Sensor.DefSensor defSensor annotation(
        Placement(visible = true, transformation(origin = {-1, 51}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
      Drone.Controller.DefController defController(Ti_c = 35, Ti_d = 1.3, k_c = 5.4) annotation(
        Placement(visible = true, transformation(origin = {0, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Propeller.DefPropeller defPropeller0 annotation(
        Placement(visible = true, transformation(origin = {-66, 34}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
      Drone.Environment.DefAir defAir annotation(
        Placement(visible = true, transformation(origin = {86, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Engine.DefEngine defEngine0 annotation(
        Placement(visible = true, transformation(origin = {-64, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Drone.Body.DefDrone defDrone annotation(
        Placement(visible = true, transformation(origin = {0, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Engine.DefEngine defEngine1 annotation(
        Placement(visible = true, transformation(origin = {66, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Drone.Propeller.DefPropeller defPropeller1 annotation(
        Placement(visible = true, transformation(origin = {67, 33}, extent = {{-27, -27}, {27, 27}}, rotation = -90)));
      Drone.Engine.DefEngine defEngine2 annotation(
        Placement(visible = true, transformation(origin = {54, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Propeller.DefPropeller defPropeller2 annotation(
        Placement(visible = true, transformation(origin = {47, -65}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
      Drone.Engine.DefEngine defEngine3 annotation(
        Placement(visible = true, transformation(origin = {-48, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Propeller.DefPropeller defPropeller3 annotation(
        Placement(visible = true, transformation(origin = {-47, -65}, extent = {{-25, -25}, {25, 25}}, rotation = -90)));
      Battery.DefBattery defBattery annotation(
        Placement(visible = true, transformation(origin = {0, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Equations --------------------
    equation
// Connectors --------------------
      connect(defHeightProfile.MeasHeightProfile, defSensor.MeasHeightProfile) annotation(
        Line(points = {{2, 94}, {20, 94}, {20, 69}, {0.5, 69}, {0.5, 55}, {-1, 55}}, color = {0, 0, 127}));
      connect(defSensor.MeasHeightProfileOut, defController.MeasHeightProfileOut) annotation(
        Line(points = {{5, 41}, {5, 32.5}, {4, 32.5}, {4, 22}}, color = {0, 0, 127}));
      connect(defSensor.DroneHeightOut, defController.DroneHeightOut) annotation(
        Line(points = {{2, 41}, {2, 23.5}}, color = {0, 0, 127}));
      connect(defController.RefEngineSpeed, defEngine0.RefEngineSpeed) annotation(
        Line(points = {{5, 17}, {-58.5, 17}, {-58.5, 10}}, color = {0, 0, 127}));
      connect(defController.RefEngineSpeed, defEngine1.RefEngineSpeed) annotation(
        Line(points = {{5, 17}, {72, 17}, {72, 10}}, color = {0, 0, 127}));
      connect(defController.RefEngineSpeed, defEngine2.RefEngineSpeed) annotation(
        Line(points = {{5, 17}, {52, 17}, {52, -34}}, color = {0, 0, 127}));
      connect(defController.RefEngineSpeed, defEngine3.RefEngineSpeed) annotation(
        Line(points = {{5, 17}, {-50, 17}, {-50, -34}}, color = {0, 0, 127}));
      connect(defAir.airDensity, defPropeller0.airDensity) annotation(
        Line(points = {{86, 86}, {-43, 86}, {-43, 57}}, color = {0, 0, 127}));
      connect(defAir.airDensity, defPropeller1.airDensity) annotation(
        Line(points = {{86, 86}, {92, 86}, {92, 8}}, color = {0, 0, 127}));
      connect(defAir.airDensity, defPropeller2.airDensity) annotation(
        Line(points = {{86, 86}, {86, -43}, {69, -43}}, color = {0, 0, 127}));
      connect(defAir.airDensity, defPropeller3.airDensity) annotation(
        Line(points = {{-24, -88}, {86, -88}, {86, 86}}, color = {0, 0, 127}));
      connect(defEngine0.phi_M_Connector, defPropeller0.phi_M_Connector) annotation(
        Line(points = {{-68, 15}, {-36.5, 15}, {-36.5, 34}, {-66, 34}}));
      connect(defEngine1.phi_M_Connector, defPropeller1.phi_M_Connector) annotation(
        Line(points = {{62, 16}, {66, 16}, {66, 32}}));
      connect(defEngine2.phi_M_Connector, defPropeller2.phi_M_Connector) annotation(
        Line(points = {{47, -44}, {47, -65}}));
      connect(defEngine3.phi_M_Connector, defPropeller3.phi_M_Connector) annotation(
        Line(points = {{-56, -44}, {-48, -44}, {-48, -66}}));
      connect(defDrone.DroneHeightOut, defSensor.DroneHeight) annotation(
        Line(points = {{-8, 12}, {-24, 12}, {-24, 50}, {-10, 50}}, color = {0, 0, 127}));
      connect(defPropeller1.s_F_Connector, defDrone.s_F_Connector1) annotation(
        Line(points = {{76, 32}, {4, 32}, {4, 4}}));
      connect(defPropeller2.s_F_Connector, defDrone.s_F_Connector2) annotation(
        Line(points = {{48, -58}, {6, -58}, {6, -2}}));
      connect(defPropeller3.s_F_Connector, defDrone.s_F_Connector3) annotation(
        Line(points = {{-40, -66}, {-4, -66}, {-4, 0}}));
      connect(defPropeller0.s_F_Connector, defDrone.s_F_Connector0) annotation(
        Line(points = {{-66, 42}, {-32, 42}, {-32, 6}, {-4, 6}}));
// assume constant speed of 14m/s for 24321.41m = 1737.24357s
      connect(defBattery.pin_pos, defEngine0.pin_pos) annotation(
        Line(points = {{-2, -16}, {-72, -16}, {-72, 8}, {-64, 8}}, color = {0, 0, 255}));
      connect(defBattery.pin_pos, defEngine3.pin_pos) annotation(
        Line(points = {{-2, -16}, {-48, -16}, {-48, -40}}, color = {0, 0, 255}));
      connect(defBattery.pin_pos, defEngine2.pin_pos) annotation(
        Line(points = {{-2, -16}, {54, -16}, {54, -40}}, color = {0, 0, 255}));
      connect(defBattery.pin_pos, defEngine1.pin_pos) annotation(
        Line(points = {{-2, -16}, {54, -16}, {54, 8}, {66, 8}}, color = {0, 0, 255}));
      connect(defBattery.pin_neg, defEngine1.pin_neg) annotation(
        Line(points = {{2, -18}, {68, -18}, {68, 6}}, color = {0, 0, 255}));
      connect(defBattery.pin_neg, defEngine2.pin_neg) annotation(
        Line(points = {{2, -18}, {56, -18}, {56, -38}}, color = {0, 0, 255}));
      connect(defBattery.pin_neg, defEngine3.pin_neg) annotation(
        Line(points = {{2, -18}, {-46, -18}, {-46, -38}}, color = {0, 0, 255}));
      connect(defBattery.pin_neg, defEngine0.pin_neg) annotation(
        Line(points = {{2, -18}, {-62, -18}, {-62, 6}}, color = {0, 0, 255}));
      connect(defBattery.pin_pos, defController.pin_pos) annotation(
        Line(points = {{-2, -16}, {-8, -16}, {-8, 18}}, color = {0, 0, 255}));
      connect(defBattery.pin_neg, defController.pin_neg) annotation(
        Line(points = {{2, -18}, {-16, -18}, {-16, 16}, {-4, 16}}, color = {0, 0, 255}));
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {2, -4}, rotation = 180, extent = {{-110, 100}, {110, -100}}, fileName = "modelica://Drone/pictures/top_view_drone_free.png")}),
        experiment(StartTime = 0, StopTime = 1737.24, Tolerance = 1e-06, Interval = 0.01),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {2, -6}, rotation = 180, extent = {{-112, 106}, {112, -106}}, fileName = "modelica://Drone/pictures/top_view_drone_free.png")}),
        Documentation);
    end DefDroneSim;
    annotation(
      Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Examples\" package contains example simualtions to show the use of the provided models.</font></p>

</body></html>"));
  end Examples;

  package Connectors
    connector Winkel_Moment_Connector
      // Variables --------------------
      Modelica.Units.SI.Angle phi "Absolute rotation angle of flange";
      flow Modelica.Units.SI.Torque tau "Cut torque in the flange";
      // Annotation --------------------
      annotation(
        Icon(graphics = {Ellipse(origin = {-1, 4}, fillColor = {37, 150, 225}, fillPattern = FillPattern.Solid, lineThickness = 1.5, extent = {{99, 94}, {-97, -102}})}, coordinateSystem(initialScale = 0.1, extent = {{-100, -100}, {100, 100}})),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br></font><p><span style=\"font-size: 14px;\"><span style=\"font-family: Verdana, Geneva, sans-serif;\">The \"Winke_Moment_Connector\" model represents&nbsp;</span></span><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">a connector for 1D rotational mechanical systems.</font></p><hr><h1><span style=\"color: rgb(41, 128, 185);\"><strong><span style=\"font-family: Verdana, Geneva, sans-serif;\">Model Structure</span></strong></span></h1><font size=\"4\"><strong>Variables:</strong></font><ul><li><font size=\"4\"><code>phi</code>: Absolute rotation angle of the flange in [rad].</font></li><li><font size=\"4\"><code>tau</code>:&nbsp;Cut-torque in the flange in [Nm].</font></li></ul></body></html>"));
    end Winkel_Moment_Connector;

    connector Weg_Kraft_Connector
      // Variables --------------------
      Modelica.Units.SI.Length s "Position";
      flow Modelica.Units.SI.Force F "Force";
      // Annotation --------------------
      annotation(
        Icon(coordinateSystem(initialScale = 0.1, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(fillColor = {0, 255, 127}, fillPattern = FillPattern.Solid, lineThickness = 1.75, points = {{0, 98}, {96, 50}, {96, -50}, {0, -98}, {-96, -50}, {-96, 50}, {0, 98}})}),
        Documentation(info = "<html><head></head><body><!--StartFragment--><br><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br></font><p><span style=\"font-size: 14px;\"><span style=\"font-family: Verdana, Geneva, sans-serif;\">The \"Weg_Kraft_Connector\" model represents&nbsp;</span></span><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">a connector for 1D translational mechanical systems.</font></p><hr><h1><span style=\"color: rgb(41, 128, 185);\"><strong><span style=\"font-family: Verdana, Geneva, sans-serif;\">Model Structure</span></strong></span></h1><font size=\"4\"><strong>Variables:</strong></font><ul><li><font size=\"4\"><code>s</code>:&nbsp;Absolute position in flange in [m].</font></li><li><font size=\"4\"><code>F</code>:&nbsp;Cut-force in direction of the flange axis in [N].</font></li></ul><!--EndFragment--></body></html>"));
    end Weg_Kraft_Connector;

    connector RealOutput = output Real "'output Real' as connector" annotation(
      defaultComponentName = "y",
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-100.0, 100.0}, {100.0, 0.0}, {-100.0, -100.0}})}),
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-100.0, 50.0}, {0.0, 0.0}, {-100.0, -50.0}}), Text(textColor = {0, 0, 127}, extent = {{30.0, 60.0}, {30.0, 110.0}}, textString = "%name")}),
      Documentation(info = "<html><head></head><body><p><br><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br></font></p><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">Connector with one output signal of type Real.</font></p>
  </body></html>"));
    connector RealInput = input Real "'input Real' as connector" annotation(
      defaultComponentName = "u",
      Icon(graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, points = {{-100.0, 100.0}, {100.0, 0.0}, {-100.0, -100.0}})}, coordinateSystem(extent = {{-100.0, -100.0}, {100.0, 100.0}}, preserveAspectRatio = true, initialScale = 0.2)),
      Diagram(coordinateSystem(preserveAspectRatio = true, initialScale = 0.2, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Polygon(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, points = {{0.0, 50.0}, {100.0, 0.0}, {0.0, -50.0}, {0.0, 50.0}}), Text(textColor = {0, 0, 127}, extent = {{-10.0, 60.0}, {-10.0, 85.0}}, textString = "%name")}),
      Documentation(info = "<html><head></head><body><p><br><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br></font></p><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">Connector with one input signal of type Real.</font></p>
  </body></html>"));
    annotation(
      Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {0, 1}, extent = {{100, -99}, {-100, 99}}, fileName = "modelica://Drone/pictures/power-cable_5770130.png")}),
      Diagram,
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Connectors\" package contains the connectors used to connect the individual components of the library</font></p>

</body></html>"));
  end Connectors;

  package Body
    model DefDrone
      //Model Definitions --------------------
      // Connectors --------------------
      Drone.Connectors.Weg_Kraft_Connector s_F_Connector0 annotation(
        Placement(visible = true, transformation(origin = {-49, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-49, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.Weg_Kraft_Connector s_F_Connector1 annotation(
        Placement(visible = true, transformation(origin = {32, 27}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {32, 27}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.Weg_Kraft_Connector s_F_Connector2 annotation(
        Placement(visible = true, transformation(origin = {53, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {53, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.Weg_Kraft_Connector s_F_Connector3 annotation(
        Placement(visible = true, transformation(origin = {-43, -21}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-43, -21}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.RealOutput DroneHeightOut(unit = "m") annotation(
        Placement(visible = true, transformation(origin = {-86, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-15, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Constants --------------------
      constant Real g = Modelica.Constants.g_n "Earth acceleration";
      // Parameters --------------------
      parameter Modelica.Units.SI.Mass mass = 4.310 "mass of drone";
      // Variables --------------------
      Modelica.Units.SI.Velocity v_y "Velocity in vertical direction (m/s)";
      Modelica.Units.SI.Acceleration a_y "Acceleration in vertical direction (m/s^2)";
      Modelica.Units.SI.Force F_y "Sum of all forces in vertical direction (N)";
      Modelica.Units.SI.Force F_g "Gravitational force (N)";
      // Equations --------------------
    equation
//Connectors --------------------
      s_F_Connector0.s = s_F_Connector1.s;
      s_F_Connector1.s = s_F_Connector2.s;
      s_F_Connector2.s = s_F_Connector3.s;
      DroneHeightOut = s_F_Connector0.s;
// Calculation of velocity, acceleration and position --------------------
      der(DroneHeightOut) = v_y;
      der(v_y) = a_y;
      F_g = mass*g;
      mass*a_y = F_y - F_g;
//Lift Force summary --------------------
      F_y = s_F_Connector0.F + s_F_Connector1.F + s_F_Connector2.F + s_F_Connector3.F;
//Ground collision prevention --------------------
      when DroneHeightOut < 0 then
        reinit(DroneHeightOut, 0.005);
        reinit(v_y, 0.005);
      end when;
// --------------------
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {-1, 0}, extent = {{99, -98}, {-98, 98}}, fileName = "modelica://Drone/pictures/food_12725960.png")}),
        Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The model \"DefDrone\" simulates the behavior of a drone in terms of its height, velocity, forces acting on it, gravitational force, and ground collision prevention. It's structured to simulate vertical motion and prevent the drone from going below ground level. </span></span></p>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Model Structure</span></strong></span></h1>


  <font size=\"4\">
    <strong>Connectors:</strong></font><ul>
    <li>
      <font size=\"4\">
        <code>s_F_Connector</code>:&nbsp;Displacement-force connector.</font></li><li><font size=\"4\"><code>RealOutput</code>: Output for&nbsp;current drone height.</font></li></ul>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameters</span></strong></span></h1>

<table>
 <thead>
  <tr>
   <th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameter</span></span></th>
   <th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Description</span></span></th>
   <th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Value</span></span></th>
  </tr>
 </thead>
 <tbody>
  <tr>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">mass</span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"> total drone mass</span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">4.31kg</span></span></td>
  </tr>
 </tbody>
</table>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Equations</span></strong></span></h1>

<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Drone position: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">d/dt(x) = v<sub>y</sub></span></span>
</p>
<p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Drone velocity v<sub>y</sub>: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">d/dt(v<sub>y</sub>) = a<sub>y</sub></span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Gravitational force: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">F<sub>g</sub> = m × g</span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Newton's second law: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">m × a<sub>y</sub> = F<sub>y</sub> - F<sub>g</sub></span></span>
</p>
</body></html>"));
    end DefDrone;
    annotation(
      Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {-1, -0.5}, extent = {{100, -99}, {-98, 100}}, fileName = "modelica://Drone/pictures/food_12725960.png")}),
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Body\" package contains the basic body of a drone which includes the technical information of the drone and calculates the drone movement.</font></p>

</body></html>"));
  end Body;

  package Propeller
    model DefPropeller
      //Model Definitions --------------------
      // Connectors --------------------
      Drone.Connectors.Winkel_Moment_Connector phi_M_Connector annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {1, -1}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.RealInput airDensity(unit = "kg/m3") annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {89, 89}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.Weg_Kraft_Connector s_F_Connector annotation(
        Placement(visible = true, transformation(origin = {-14, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {1, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Constants --------------------
      constant Real g = Modelica.Constants.g_n "Gravitational force (N)";
      // Parameters --------------------
      parameter Modelica.Units.SI.Length rProp = 0.203 "propeller radius";
      parameter Real C_l = 0.28 "Lift coefficient";
      parameter Real C_w = 0.09 "Drag coefficient";
      parameter Real NumProp = 1 "Number of propellers";
      // Variables --------------------
      Modelica.Units.SI.Force F_lp "Drone lift force per propeller";
      Modelica.Units.SI.Force F_l "Drone lift force";
      Modelica.Units.SI.Area AProp "Propeller area";
      Modelica.Units.SI.AngularVelocity propOmega "Propeller angular velocity";
      Real rpmProp(unit = "rpm");
      // Equations --------------------
    equation
//Lift force and RPM of the Propeller
      propOmega = der(phi_M_Connector.phi);
      AProp = Modelica.Constants.pi*rProp^2;
      F_lp = 0.5*airDensity*((rProp*propOmega)^2)*AProp*C_l;
      F_l = F_lp*NumProp;
      phi_M_Connector.tau = rProp*0.5*C_w*airDensity*AProp*(rProp*propOmega)^2;
      rpmProp = Modelica.Units.Conversions.to_rpm(propOmega);
      0 = s_F_Connector.F + F_l;
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(extent = {{-100, 100}, {-100, 100}}), Bitmap(origin = {-0.5, -1}, extent = {{-98, 99}, {99, -98}}, fileName = "modelica://Drone/pictures/propeller_6275206.png")}),
        Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The \"DefPropeller\" model represents the dynamics of a single drone propeller system. It calculates the lift force generated by the propellers based on various factors like air density, propeller radius, lift and drag coefficients, angular velocity, and the number of propellers. The model ensures force equilibrium by considering the sum of forces acting on the propeller system. </span></span></p>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Model Structure</span></strong></span>

    
  </h1>
  <font size=\"4\">
    <strong>Connectors:</strong></font><ul>
    <li>
      <font size=\"4\">
        <code>s_F_Connector</code>:&nbsp;Displacement-force connector.
      </font>
    </li>
    <li>
      <font size=\"4\">
        <code>phi_M_Connector</code>: Angle of rotation and moment connector.</font></li><li><font size=\"4\"><code>RealInput</code>: Input for air density.</font></li></ul>


<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameters</span></strong></span></h1>


<table>
 <thead>
  <tr>
   <th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameter</span></span></th>
   <th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Description</span></span></th>
   <th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Value</span></span></th>
  </tr>
 </thead>
 <tbody>
  <tr>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">C<sub>l</sub></span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"> Lift coefficient</span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">0.34</span></span></td>
  </tr>
  <tr>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">C<sub>w</sub></span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Drag coefficient</span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">0.05</span></span></td>
  </tr>
  <tr>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">NumProp</span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"> Number of propellers</span></span></td>
   <td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">1</span></span></td>
  </tr>
 </tbody>
</table>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Equations</span></strong></span></h1>

<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Angular velocity of the propeller: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">ω = d/dt(φ)</span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Area of the propeller: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">A = π × r<sup>2</sup></span></span></p>
<p>
 <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Lift Force per propeller:</span></span>
 <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">F<sub>lp</sub> = 0.5 × ρ × (r × ω)<sup>2</sup> × A<sub>Prop</sub> × C<sub>l</sub></span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Absolute Lift Force </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">F<sub>l</sub> = F<sub>lp</sub> × NumProp</span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Air resistance: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">τ = r × 0.5 × C<sub>w</sub> × ρ × A<sub>Prop</sub> × (r × ω)<sup>2</sup></span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Rad/s to RPM: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"> RPM = (rad/s) × 60 / (2π)</span></span>
</p>
<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Sum of the connector forces:</span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">  0 = F<sub>connector</sub> + F<sub>l</sub></span></span>
</p>
</body></html>"));
    end DefPropeller;

    // Annotation --------------------
    annotation(
      Icon(graphics = {Bitmap(extent = {{-98, 98}, {98, -98}}, imageSource = "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")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
      Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Propeller\" package contain models with propellers that can be used in combination with engines to generate lift force.</font></p>

</body></html>"));
  end Propeller;

  package Engine
    model DefEngine "Speed controlled DC PM drive with H-bridge from battery"
      //  extends Utilities.PartialControlledDCPM;
      // Parameters --------------------
      parameter Modelica.Units.SI.Voltage ViNominal = dcpmData.VaNominal - Ra*dcpmData.IaNominal "Nominal induced voltage";
      parameter Modelica.Units.SI.Time Ta = dcpmData.La/Ra "Armature time constant";
      parameter Modelica.Units.SI.Time Ts = 1e-4 "Dead time of inverter";
      parameter Modelica.Units.SI.Resistance k = Ra*Ta/(2*Ts) "Current controller proportional gain";
      parameter Modelica.Units.SI.Time Ti = Ta "Current controller integral time constant";
      parameter Modelica.Units.SI.MagneticFlux kPhi = ViNominal/dcpmData.wNominal "Voltage constant";
      parameter Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.DriveDataDCPM driveData(VBat = 23.1, fS = 4e3, motorData = dcpmData) annotation(
        Placement(visible = true, transformation(origin = {58, 138}, extent = {{20, -80}, {40, -60}}, rotation = 0)));
      parameter Modelica.Units.SI.Torque TLoad = ViNominal*dcpmData.IaNominal/dcpmData.wNominal "Nominal load torque";
      parameter Modelica.Units.SI.AngularVelocity wLoad = dcpmData.wNominal "Nominal load torque";
      parameter Modelica.Units.SI.Inertia JLoad = dcpmData.Jr "Load's moment of inertia";
      parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData dcpmData(Jr = 10e-3, Js = 10e-3, La(displayUnit = "uH") = 9.050000000000007e-06, Ra(displayUnit = "mOhm") = 0.1542, VaNominal = 23.1, wNominal = 1800*2*Modelica.Constants.pi/60) annotation(
        Placement(visible = true, transformation(origin = {78, 140}, extent = {{0, -60}, {20, -40}}, rotation = 0)));
      parameter Modelica.Units.SI.Resistance Ra = Modelica.Electrical.Machines.Thermal.convertResistance(dcpmData.Ra, dcpmData.TaRef, dcpmData.alpha20a, dcpmData.TaNominal) "Warm armature resistance";
      Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(IaNominal = dcpmData.IaNominal, Jr(fixed = true) = dcpmData.Jr, Js = dcpmData.Js, La = dcpmData.La, Ra = dcpmData.Ra, TaNominal = dcpmData.TaNominal, TaOperational = 300.15, TaRef = dcpmData.TaRef, VaNominal = dcpmData.VaNominal, alpha20a = dcpmData.alpha20a, coreParameters = driveData.motorData.coreParameters, frictionParameters = driveData.motorData.frictionParameters, ia(fixed = true), phiMechanical(displayUnit = "rad", fixed = true), useSupport = false, wMechanical(displayUnit = "rad/s", fixed = true), wNominal = dcpmData.wNominal) annotation(
        Placement(visible = true, transformation(origin = {16, -20}, extent = {{0, -30}, {20, -10}}, rotation = 0)));
      // Connectors --------------------
      Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI speedController(Ti = driveData.Tiw, constantLimits = true, initType = Modelica.Blocks.Types.Init.InitialOutput, k = driveData.kpw, symmetricLimits = false, yMin = 0) annotation(
        Placement(visible = true, transformation(origin = {-10, 48}, extent = {{-120, -20}, {-100, 0}}, rotation = 0)));
      Modelica.Blocks.Math.Gain tau2i(k = 1/driveData.kPhi) annotation(
        Placement(visible = true, transformation(origin = {-80, 38}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
      Drone.Connectors.Winkel_Moment_Connector phi_M_Connector annotation(
        Placement(visible = true, transformation(origin = {90, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-74, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.RealInput RefEngineSpeed annotation(
        Placement(visible = true, transformation(origin = {-200, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-20, 55}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
      Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI currentController(KFF = driveData.kPhi, Ti = driveData.TiI, constantLimits = true, initType = Modelica.Blocks.Types.Init.InitialOutput, k = driveData.kpI, symmetricLimits = false, useFF = true) annotation(
        Placement(visible = true, transformation(origin = {10, 48}, extent = {{-50, -20}, {-30, 0}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = driveData.aMax, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
        Placement(visible = true, transformation(origin = {-162, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_neg annotation(
        Placement(visible = true, transformation(origin = {16, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {16, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_pos annotation(
        Placement(visible = true, transformation(origin = {48, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-4, 9}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Sensors --------------------
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
      // Inverter --------------------
      Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.DcdcInverter armatureInverter(Td = driveData.Td, Tmf = driveData.Tmf, VMax = driveData.VaMax, fS = driveData.fS) annotation(
        Placement(visible = true, transformation(origin = {-6, 48}, extent = {{20, -20}, {40, 0}}, rotation = 0)));
      // Equations --------------------
    equation
// Connectors --------------------
      connect(speedSensor.w, speedController.u_m) annotation(
        Line(points = {{50, -81}, {50, -90}, {-126, -90}, {-126, 26}}, color = {0, 0, 127}));
      connect(speedController.y, tau2i.u) annotation(
        Line(points = {{-108, 38}, {-92, 38}}, color = {0, 0, 127}));
      connect(dcpm.flange, phi_M_Connector) annotation(
        Line(points = {{36, -40}, {90, -40}}));
      connect(dcpm.flange, speedSensor.flange) annotation(
        Line(points = {{36, -40}, {50, -40}, {50, -60}}));
      connect(RefEngineSpeed, slewRateLimiter.u) annotation(
        Line(points = {{-200, 38}, {-174, 38}}, color = {0, 0, 127}));
      connect(slewRateLimiter.y, speedController.u) annotation(
        Line(points = {{-151, 38}, {-132, 38}}, color = {0, 0, 127}));
      connect(tau2i.y, currentController.u) annotation(
        Line(points = {{-68, 38}, {-42, 38}}, color = {0, 0, 127}));
      connect(armatureInverter.vDC, currentController.yMaxVar) annotation(
        Line(points = {{14, 44}, {-18, 44}}, color = {0, 0, 127}));
      connect(armatureInverter.iMot, currentController.u_m) annotation(
        Line(points = {{14, 32}, {0, 32}, {0, 10}, {-36, 10}, {-36, 26}}, color = {0, 0, 127}));
      connect(currentController.y, armatureInverter.vRef) annotation(
        Line(points = {{-18, 38}, {12, 38}}, color = {0, 0, 127}));
      connect(armatureInverter.pin_nMot, dcpm.pin_an) annotation(
        Line(points = {{18, 28}, {20, 28}, {20, -30}}, color = {0, 0, 255}));
      connect(armatureInverter.pin_pMot, dcpm.pin_ap) annotation(
        Line(points = {{30, 28}, {32, 28}, {32, -30}}, color = {0, 0, 255}));
      connect(speedSensor.w, currentController.feedForward) annotation(
        Line(points = {{50, -80}, {50, -90}, {-30, -90}, {-30, 26}}, color = {0, 0, 127}));
      connect(pin_pos, armatureInverter.pin_pBat) annotation(
        Line(points = {{48, 84}, {48, 60}, {30, 60}, {30, 48}}, color = {0, 0, 255}));
      connect(pin_neg, armatureInverter.pin_nBat) annotation(
        Line(points = {{16, 84}, {18, 84}, {18, 48}}, color = {0, 0, 255}));
// Annotation --------------------
      annotation(
        experiment(StopTime = 150, Interval = 0.3, StartTime = 0, Tolerance = 1e-06),
        Documentation(info = "<html>
<head></head>
<body>
  <font size=\"6\">
    <strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">
      Model Overview
    </strong>
    <br>
  </font>
  <p>
    <!--StartFragment-->
    <font size=\"4\">
      <span style=\"font-family: 'Segoe WPC', 'Segoe UI', sans-serif; font-variant-ligatures: normal; orphans: 2; widows: 2; background-color: rgb(255, 255, 255);\">
        This Modelica model represents a speed-controlled DC PM drive utilizing an H-bridge configuration from a battery. It simulates the behavior of an electrical machine, specifically a drone motor, by managing various parameters and control mechanisms.
      </span>
      &nbsp;
    </font>
  </p>

  <hr>
  <h1>
    <span style=\"color:#2980b9\">
      <strong>
        <span style=\"font-family:Verdana,Geneva,sans-serif\">Model Structure</span>
      </strong>
    </span>
  </h1>
  <font size=\"4\">
    <strong>Parameters:</strong>
  </font>
  <!-- Include specific information about the parameters and their significance in the model -->

  <hr>
  <h1>
    <span style=\"color:#2980b9\">
      <strong>
        <span style=\"font-family:Verdana,Geneva,sans-serif\">Equations</span>
      </strong>
    </span>
  </h1>
  <div>
    <p>
      <font size=\"4\">
        This electrical machine model is governed by complex connections, control mechanisms, and equations that manage torque, speed, current, and voltage interactions within the system.
      </font>
    </p>
  </div>
  <!-- You might want to add more information regarding the equations specific to the electrical machine if available -->
</body>
</html>"),
        Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}})),
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(rotation = 180, extent = {{-99, 99}, {99, -99}}, fileName = "modelica://Drone/pictures/electric-motor_5770153.png")}));
    end DefEngine;

    // Annotation --------------------
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(rotation = 180, extent = {{-99, 99}, {99, -99}}, fileName = "modelica://Drone/pictures/electric-motor_5770153.png")}),
      Diagram,
      experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.02),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Engine\" package contains models for engines that can be used to power the drones propellers.</font></p>

</body></html>"));
  end Engine;

  package Battery
    model DefBattery
      // Imports --------------------
      Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(Np = 3, Ns = 8, SOC(fixed = true, start = 0.96), cellData = cellData) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Connectors --------------------
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_pos annotation(
        Placement(visible = true, transformation(origin = {-56, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-20, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_neg annotation(
        Placement(visible = true, transformation(origin = {54, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {23, 59}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Parameters --------------------
      parameter Modelica.Electrical.Batteries.ParameterRecords.CellData cellData(OCVmax = 3.8, OCVmin = 2.3, Qnom = 15408, Ri = cellData.OCVmax/500) annotation(
        Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Equations --------------------
    equation
// --------------------
// Connectors --------------------
      connect(cellStack.p, pin_pos) annotation(
        Line(points = {{-10, 0}, {-56, 0}}, color = {0, 0, 255}));
      connect(cellStack.n, pin_neg) annotation(
        Line(points = {{10, 0}, {54, 0}}, color = {0, 0, 255}));
      connect(cellStack.n, ground.p) annotation(
        Line(points = {{10, 0}, {30, 0}, {30, -20}}, color = {0, 0, 255}));
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {0.5, 0}, rotation = 180, extent = {{-99, 100}, {100, -100}}, fileName = "modelica://Drone/pictures/power-bank_5735222.png")}),
        experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-6, Interval = 0.04),
        Documentation(info = "<html><head></head>
<body>
  <font size=\"6\">
    <strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">
      Model Overview
    </strong>
    <br></font><p>
    <!--StartFragment-->
    
      <span style=\"font-size: large;\">This Modelica model represents a battery system composed of cell stacks. It simulates the behavior of battery cells within the stack, managing the voltage, state of charge, and other parameters.</span>
    </font>
  </p>

  <hr>
  <h1>
    <span style=\"color:#2980b9\">
      <strong>
        <span style=\"font-family:Verdana,Geneva,sans-serif\">Model Structure</span>
      </strong>
    </span>
  </h1>
  <font size=\"4\">
    <strong>Connectors:</strong></font><ul>
    <li>
      <font size=\"4\">
        <code>pin_pos</code>: Electrical interface positive pin.
      </font>
    </li>
    <li>
      <font size=\"4\">
        <code>pin_neg</code>: Electrical interface negative pin.
      </font>
    </li>
  </ul><div><font size=\"4\"><b>Blocks:&nbsp;</b></font></div><div><ul><li><font size=\"4\"><code>cellStack</code>: Represents the battery cell stack composed of multiple cells.</font></li><li><font size=\"4\"><code>ground</code>: Electrical ground.</font><!--EndFragment--></li></ul></div>

  <hr>
  <h1>
    <span style=\"color:#2980b9\">
      <strong>
        <span style=\"font-family:Verdana,Geneva,sans-serif\">Parameters</span>
      </strong>
    </span>
  </h1>
  <font size=\"4\">
    <strong>Parameters:</strong>
  </font>
  <ul>
    <li>
      <font size=\"4\">
        <code>cellData</code>: Defines characteristics of battery cells (OCVmax, OCVmin, Qnom, Ri).
      </font>
    </li>
  </ul>

  <hr>
  <h1>
    <span style=\"color:#2980b9\">
      <strong>
        <span style=\"font-family:Verdana,Geneva,sans-serif\">Equations</span>
      </strong>
    </span>
  </h1>
  <div>
    <p>
      <font size=\"4\">
        This battery model is governed by electrical connections and cell behavior equations that manage the flow of current and voltage across the stack.
      </font>
    </p>
  </div>
  <!-- You might want to add more information regarding the equations specific to the battery behavior if available -->


</body></html>"));
    end DefBattery;

    // Equations --------------------
  equation
// Annotation --------------------
    annotation(
      Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {0.5, 0}, rotation = 180, extent = {{-99, 100}, {100, -100}}, fileName = "modelica://Drone/pictures/power-bank_5735222.png")}),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Battery\" package contains battery models that can be used to power the drone.</font></p>

</body></html>"));
  end Battery;

  package Controller
    model DefController
      // Model Definitions --------------------
      // Connectors --------------------
      Drone.Connectors.RealOutput RefEngineSpeed annotation(
        Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {51, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.RealInput DroneHeightOut annotation(
        Placement(visible = true, transformation(origin = {44, 90}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {18, 55}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Drone.Connectors.RealInput MeasHeightProfileOut annotation(
        Placement(visible = true, transformation(origin = {66, 90}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {42, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_pos annotation(
        Placement(visible = true, transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-74, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_neg annotation(
        Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-42, -19}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Blocks --------------------
      Modelica.Blocks.Continuous.LimPID pid(Td = Ti_d, Ti = Ti_c, k = k_c, withFeedForward = true, yMax = 900, yMin = 0) annotation(
        Placement(visible = true, transformation(origin = {0, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R(displayUnit = "Ohm") = 10, T_ref = 313.15) annotation(
        Placement(visible = true, transformation(origin = {2, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Parameters --------------------
      parameter Real k_c = 2;
      parameter Real Ti_c = 10;
      parameter Real Ti_d = 0.8;
      // Variables --------------------
      Real rpmRef(unit = "rpm");
      // Equations --------------------
    equation
// Connectors --------------------
      connect(DroneHeightOut, pid.u_m) annotation(
        Line(points = {{44, 90}, {44, 18}, {0, 18}, {0, 24}}, color = {0, 0, 127}));
      connect(MeasHeightProfileOut, pid.u_ff) annotation(
        Line(points = {{66, 90}, {66, 18}, {6, 18}, {6, 24}}, color = {0, 0, 127}));
      connect(MeasHeightProfileOut, pid.u_s) annotation(
        Line(points = {{66, 90}, {66, -50}, {-30, -50}, {-30, 36}, {-12, 36}}, color = {0, 0, 127}));
      connect(pid.y, RefEngineSpeed) annotation(
        Line(points = {{12, 36}, {30, 36}, {30, 0}, {90, 0}}, color = {0, 0, 127}));
      connect(resistor.p, pin_pos) annotation(
        Line(points = {{-8, -70}, {-50, -70}}, color = {0, 0, 255}));
      connect(resistor.n, pin_neg) annotation(
        Line(points = {{12, -70}, {70, -70}}, color = {0, 0, 255}));
// Equation rad/s to rpm --------------------
      rpmRef = Modelica.Units.Conversions.to_rpm(RefEngineSpeed);
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {1, 0}, rotation = 180, extent = {{-99, 99}, {99, -99}}, fileName = "modelica://Drone/pictures/motherboard_2656219.png")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Model Overview</strong><br>

</font><p><!--StartFragment--><font size=\"4\"><span style=\"font-family: 'Segoe WPC', 'Segoe UI', sans-serif; font-variant-ligatures: normal; orphans: 2; widows: 2; background-color: rgb(255, 255, 255);\">This Modelica model represents the flight controller for a drone designed to execute contour flight maneuvers. It manages the drone's engine speed based on measured height profiles and reference inputs.</span>&nbsp;</font></p>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Model Structure</span></strong></span></h1>

<font size=\"4\"><strong>Connectors:</strong>
    </font><ul>
      <li><font size=\"4\"><code>RefEngineSpeed</code>: RealOutput connector for reference engine speed.</font></li>
      <li><font size=\"4\"><code>DroneHeightOut</code>: RealInput connector for drone height output.</font></li>
      <li><font size=\"4\"><code>MeasHeightProfileOut</code>: RealInput connector for measured height profile output.</font></li>
      <li><font size=\"4\"><code>pin_pos</code>: Electrical interface positive pin.</font></li>
      <li><font size=\"4\"><code>pin_neg</code>: Electrical interface negative pin.</font></li>
    </ul>
    
    <font size=\"4\"><strong>Blocks:</strong>
    </font><ul>
      <li><font size=\"4\"><code>pid</code>: Continuous PID controller with specified parameters.</font></li>
      <li><font size=\"4\"><code>resistor</code>: Basic electrical resistor with resistance set to 30 Ohms, used to simulate power consumption by controller, sensor, cameras, ...</font></li></ul>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameters</span></strong></span></h1>

<font size=\"4\"><strong>Parameters:</strong></font><ul><li><font size=\"4\"><code>k_c</code>: Proportional gain.</font></li><li><font size=\"4\"><code>Ti_c</code>: Integral time constant.</font></li><li><font size=\"4\"><code>Ti_d</code>: Derivative time constant.</font></li></ul><font size=\"4\"><strong>Variables:</strong></font><ul><li><font size=\"4\"><code>rpmRef</code>: Real variable representing engine speed in RPM (revolutions per minute) for better model validation.</font></li></ul>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Equations</span></strong></span></h1><div><p><font size=\"4\">The general form of a PID controller equation is:</font></p><p style=\"font-family: 'Courier New', monospace;\"><font size=\"4\">y(t) = Kp * e(t) + Ki * ∫(e(t) dt) + Kd * d(e(t))/dt</font></p><p><font size=\"4\">Where:</font></p><ul><li><font size=\"4\"><code>y(t)</code>: Controller output at time&nbsp;<em>t</em>.</font></li><li><font size=\"4\"><code>e(t)</code>: Deviation or error at time&nbsp;<em>t</em>&nbsp;(difference between desired setpoint and measured value).</font></li><li><font size=\"4\"><code>Kp</code>: Proportional gain.</font></li><li><font size=\"4\"><code>Ki</code>: Integral gain.</font></li><li><font size=\"4\"><code>Kd</code>: Derivative gain.</font></li><li><font size=\"4\">∫: Represents integration.</font></li><li><font size=\"4\">d/dt: Represents differentiation with respect to time.</font></li></ul><p><font size=\"4\">This equation adjusts the controller's output based on the proportional, integral, and derivative terms to minimize the error and regulate the system.</font></p></div></body></html>"));
    end DefController;
  equation

    annotation(
      Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(rotation = 180, extent = {{-99, 99}, {99, -99}}, fileName = "modelica://Drone/pictures/motherboard_2656219.png")}),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Controller\" package contains models for controllers that can be used to regulate the power supply, speed and height of the drones engine.</font></p>

</body></html>"));
  end Controller;

  package Sensor
    model DefSensor
      // Model Definitions --------------------
      // Connectors --------------------
      Drone.Connectors.RealInput MeasHeightProfile annotation(
        Placement(visible = true, transformation(origin = {0, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-0.5, 32.5}, extent = {{-11.5, -11.5}, {11.5, 11.5}}, rotation = 90)));
      Drone.Connectors.RealInput DroneHeight annotation(
        Placement(visible = true, transformation(origin = {-34, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Drone.Connectors.RealOutput MeasHeightProfileOut annotation(
        Placement(visible = true, transformation(origin = {58, -84}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {58, -87}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Drone.Connectors.RealOutput DroneHeightOut annotation(
        Placement(visible = true, transformation(origin = {30, -84}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {29, -87}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      // Blocks --------------------
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {10, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 5, height = 20, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-60, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Equations --------------------
    equation
// Connectors --------------------
      connect(DroneHeight, DroneHeightOut) annotation(
        Line(points = {{-34, -42}, {30, -42}, {30, -84}}, color = {0, 0, 127}));
      connect(MeasHeightProfile, add.u1) annotation(
        Line(points = {{0, 80}, {-26, 80}, {-26, 8}, {-2, 8}}, color = {0, 0, 127}));
      connect(add.y, MeasHeightProfileOut) annotation(
        Line(points = {{22, 2}, {58, 2}, {58, -84}}, color = {0, 0, 127}));
      connect(ramp.y, add.u2) annotation(
        Line(points = {{-48, -4}, {-2, -4}}, color = {0, 0, 127}));
// Annotation -------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {1, -1}, rotation = 180, extent = {{-97, 97}, {97, -97}}, fileName = "modelica://Drone/pictures/sensor.png")}),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The provided Model \"DefSensor\" representing a sensor system within a drone. This model consists of connectors, blocks, and equations to manage measured height profile data and the drone's height. </span></span></p>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Model Structure</span></strong></span></h1>

<p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The model includes 4 connectors to meassure the height of the Drone and the distance to the ground. It also has 2 of the 4 connectors to give out the scanned informations.</span></span></p>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameters</span></strong></span></h1>

<table>
	<thead>
		<tr>
			<th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameter</span></span></th>
			<th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Description</span></span></th>
			<th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Value</span></span></th></th>
		</tr>
	</thead>
	<tbody>
		<tr>
			<td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">height</span></span></td>
			<td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"> height difference in ramp block</span></span></td>
			<td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">20m</span></span></td>
		</tr>
	</tbody>
</table>
<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Equations</span></strong></span></h1>

<p>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Height difference: </span></span>
    <span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">should_height = measured_height + height_diff</span></span>
</p>

</body></html>"));
    end DefSensor;

    // Annotation --------------------
    annotation(
      Icon(graphics = {Bitmap(rotation = 180, extent = {{-97, 97}, {97, -97}}, fileName = "modelica://Drone/pictures/sensor.png")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">The \"Sensor\" package contains models for sensors that can be used for the topological flight of a drone.</font></p>

</body></html>"));
  end Sensor;

  package Environment
    //Model DefHeightProfile --------------------

    model DefHeightProfile
      // Model Definitions --------------------
      // Connectors --------------------
      Drone.Connectors.RealOutput MeasHeightProfile(unit = "m") annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {29, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Height profile as timeTable --------------------
      Modelica.Blocks.Sources.TimeTable timeTable(table = [0.00, 0.00; 45.60, 0.01; 109.67, 0.02; 173.71, 0.03; 237.69, 0.04; 301.61, 5.48; 365.40, 7.86; 461.24, 8.79; 525.20, 10.03; 621.10, 11.58; 684.94, 11.58; 748.78, 11.58; 812.68, 12.20; 876.57, 12.82; 940.47, 13.44; 1004.31, 13.44; 1068.09, 12.82; 1132.08, 14.37; 1196.00, 15.30; 1259.84, 15.30; 1323.68, 15.30; 1387.55, 15.61; 1451.42, 15.92; 1515.17, 14.99; 1578.93, 14.06; 1626.75, 13.44; 1674.80, 15.30; 1738.76, 16.54; 1786.75, 17.78; 1866.78, 20.26; 1930.79, 22.12; 1994.69, 22.74; 2058.18, 19.02; 2089.85, 16.23; 2153.60, 15.30; 2217.53, 16.23; 2281.37, 16.23; 2345.00, 14.06; 2408.82, 13.75; 2472.66, 13.75; 2536.58, 14.68; 2600.48, 15.30; 2664.35, 15.61; 2728.30, 16.85; 2792.17, 17.16; 2856.01, 17.16; 2919.91, 17.78; 2983.75, 17.78; 3047.70, 19.02; 3111.54, 19.02; 3175.50, 20.26; 3239.36, 20.57; 3303.29, 21.50; 3367.22, 22.43; 3431.08, 22.74; 3494.92, 22.74; 3558.76, 22.74; 3622.66, 23.36; 3686.56, 23.98; 3750.40, 23.98; 3814.27, 24.29; 3878.25, 25.84; 3942.15, 26.46; 4005.99, 26.46; 4069.85, 26.77; 4133.72, 27.08; 4197.62, 27.70; 4261.49, 28.01; 4325.41, 28.94; 4389.25, 28.94; 4453.18, 29.87; 4517.19, 31.73; 4581.17, 33.28; 4645.13, 34.52; 4709.14, 36.38; 4773.15, 38.24; 4837.14, 39.79; 4901.12, 41.34; 4965.30, 45.06; 5029.34, 47.23; 5093.38, 49.40; 5157.28, 50.02; 5221.23, 51.26; 5285.16, 52.19; 5349.14, 53.74; 5412.98, 53.74; 5428.71, 51.26; 5476.82, 53.74; 5540.95, 56.84; 5589.06, 59.32; 5621.15, 61.18; 5669.23, 63.35; 5717.25, 64.90; 5797.11, 65.52; 5861.24, 68.62; 5909.29, 70.48; 5957.51, 74.20; 5989.69, 76.99; 6027.15, 79.37; 6085.94, 82.26; 6134.05, 84.74; 6182.18, 87.53; 6198.40, 90.32; 6246.74, 95.28; 6230.55, 92.80; 6278.95, 98.38; 6343.10, 101.79; 6359.26, 103.96; 6407.52, 107.99; 6391.35, 105.82; 6439.66, 110.47; 6472.01, 115.12; 6455.82, 112.64; 6504.22, 118.22; 6568.52, 123.18; 6552.33, 120.70; 6600.75, 126.59; 6616.91, 128.76; 6649.18, 132.48; 6649.01, 130.62; 6681.44, 136.20; 6681.27, 134.34; 6713.59, 138.68; 6745.74, 141.16; 6777.89, 143.64; 6831.45, 147.57; 6809.98, 145.50; 6890.33, 151.39; 6922.50, 154.18; 6970.67, 157.28; 7002.93, 161.00; 7051.07, 163.79; 7067.23, 165.96; 7099.61, 170.92; 7099.38, 168.44; 7147.75, 173.71; 7179.93, 176.50; 7244.05, 179.60; 7308.01, 180.84; 7371.96, 182.08; 7419.90, 182.70; 7483.91, 184.56; 7516.06, 187.04; 7548.21, 189.52; 7596.37, 192.62; 7644.43, 194.48; 7692.51, 196.65; 7756.38, 196.96; 7820.27, 197.58; 7884.11, 197.58; 7947.84, 196.34; 8011.62, 195.72; 8075.29, 193.86; 8059.21, 192.62; 8106.92, 190.76; 8170.67, 189.83; 8202.31, 186.73; 8266.06, 185.80; 8329.93, 186.11; 8393.68, 185.18; 8457.47, 184.56; 8521.36, 185.18; 8585.20, 185.18; 8648.99, 184.56; 8712.77, 183.94; 8760.65, 183.94; 8808.30, 181.46; 8872.02, 180.22; 8935.75, 178.98; 8999.36, 176.50; 9062.97, 174.02; 9094.55, 170.30; 9094.38, 168.44; 9142.14, 167.20; 9157.87, 164.72; 9189.50, 161.62; 9173.43, 160.38; 9269.48, 163.48; 9317.50, 165.03; 9381.25, 164.10; 9444.86, 161.62; 9492.57, 159.76; 9508.33, 157.59; 9524.09, 155.42; 9572.11, 156.97; 9635.95, 156.97; 9699.82, 157.28; 9763.63, 156.97; 9827.50, 157.28; 9891.48, 158.83; 9955.32, 158.83; 10019.02, 157.28; 10082.78, 156.35; 10146.56, 155.73; 10210.25, 154.18; 10273.95, 152.63; 10337.53, 149.84; 10401.32, 149.22; 10465.15, 149.22; 10528.99, 149.22; 10592.83, 149.22; 10656.79, 150.46; 10720.69, 151.08; 10784.50, 150.77; 10848.25, 149.84; 10911.80, 146.74; 10975.42, 144.26; 11039.14, 143.02; 11102.98, 143.02; 11166.82, 143.02; 11230.66, 143.02; 11294.36, 141.47; 11358.22, 141.78; 11422.27, 143.95; 11486.16, 144.57; 11550.09, 145.50; 11613.93, 145.50; 11677.83, 146.12; 11741.78, 147.36; 11805.68, 147.98; 11869.66, 149.53; 11933.59, 150.46; 11997.54, 151.70; 12061.52, 153.25; 12125.53, 155.11; 12189.52, 156.66; 12253.47, 157.90; 12301.41, 158.52; 12333.56, 161.00; 12381.72, 164.10; 12445.68, 165.34; 12514.93, 166.37; 12573.36, 165.34; 12621.07, 163.48; 12636.83, 161.31; 12700.55, 160.07; 12764.42, 160.38; 12812.47, 162.24; 12860.64, 165.34; 12924.79, 168.75; 12908.63, 166.58; 12988.60, 168.44; 13020.38, 166.89; 13068.29, 167.20; 13116.34, 169.06; 13180.27, 169.99; 13244.14, 170.30; 13339.78, 169.06; 13403.71, 169.99; 13467.55, 169.99; 13531.19, 167.82; 13515.11, 166.58; 13595.00, 167.51; 13658.98, 169.06; 13722.79, 168.75; 13770.58, 167.82; 13818.29, 165.96; 13850.01, 163.79; 13881.70, 161.31; 13977.26, 159.14; 14041.16, 159.76; 14089.10, 160.38; 14168.92, 160.69; 14232.76, 160.69; 14296.46, 159.14; 14280.39, 157.90; 14360.16, 157.59; 14408.01, 157.28; 14423.80, 155.42; 14487.55, 154.49; 14551.33, 153.87; 14615.00, 152.01; 14678.78, 151.39; 14742.57, 150.77; 14806.26, 149.22; 14870.10, 149.22; 14933.89, 148.60; 14997.64, 147.67; 15061.42, 147.05; 15109.22, 146.12; 15108.99, 143.64; 15124.60, 139.92; 15140.33, 137.44; 15140.16, 135.58; 15188.36, 138.99; 15236.15, 138.06; 15283.86, 136.20; 15299.65, 134.34; 15331.97, 138.68; 15331.80, 136.82; 15385.53, 142.61; 15364.06, 140.54; 15427.96, 141.16; 15459.70, 139.30; 15507.35, 136.82; 15539.02, 134.03; 15586.75, 132.48; 15634.43, 130.31; 15698.07, 128.14; 15745.78, 126.28; 15793.37, 123.18; 15841.08, 121.32; 15856.84, 119.15; 15888.56, 116.98; 15952.29, 115.74; 15952.11, 113.88; 16015.78, 112.02; 16052.77, 109.33; 16111.08, 107.06; 16095.01, 105.82; 16174.58, 103.34; 16238.19, 100.86; 16285.78, 97.76; 16301.54, 95.59; 16338.56, 93.21; 16396.87, 90.94; 16428.53, 88.15; 16460.22, 85.67; 16491.92, 83.19; 16555.53, 80.71; 16587.27, 78.85; 16618.94, 76.06; 16650.51, 72.34; 16650.34, 70.48; 16714.09, 69.55; 16777.79, 68.00; 16841.63, 68.00; 16905.36, 66.76; 16969.14, 66.14; 16953.06, 64.90; 17032.89, 65.21; 17096.65, 64.28; 17160.43, 63.66; 17224.21, 63.04; 17288.02, 62.73; 17351.78, 61.80; 17383.55, 60.25; 17447.45, 60.87; 17511.20, 59.94; 17495.13, 58.70; 17574.90, 58.39; 17638.71, 58.08; 17702.46, 57.15; 17766.28, 56.84; 17830.03, 55.91; 17893.78, 54.98; 17957.57, 54.36; 18021.38, 54.05; 18085.13, 53.12; 18148.91, 52.50; 18212.67, 51.57; 18276.48, 51.26; 18340.32, 51.26; 18404.04, 50.02; 18467.85, 49.71; 18531.58, 48.47; 18595.36, 47.85; 18659.15, 47.23; 18722.90, 46.30; 18786.74, 46.30; 18850.46, 45.06; 18914.28, 44.75; 18978.06, 44.13; 19041.81, 43.20; 19105.59, 42.58; 19169.43, 42.58; 19233.27, 42.58; 19297.06, 41.96; 19360.87, 41.65; 19424.65, 41.03; 19488.46, 40.72; 19552.27, 40.41; 19616.05, 39.79; 19679.84, 39.17; 19743.65, 38.86; 19807.40, 37.93; 19871.21, 37.62; 19934.94, 36.38; 19998.72, 35.76; 20062.53, 35.45; 20126.32, 34.83; 20190.10, 34.21; 20253.88, 33.59; 20317.69, 33.28; 20381.47, 32.66; 20445.31, 32.66; 20509.10, 32.04; 20572.82, 30.80; 20636.60, 30.18; 20700.44, 30.18; 20764.28, 30.18; 20828.12, 30.18; 20891.96, 30.18; 20955.77, 29.87; 21019.59, 29.56; 21083.37, 28.94; 21147.15, 28.32; 21210.93, 27.70; 21274.77, 27.70; 21338.58, 27.39; 21402.40, 27.08; 21466.23, 27.08; 21530.07, 27.08; 21593.89, 26.77; 21657.67, 26.15; 21721.45, 25.53; 21785.18, 24.29; 21849.04, 24.60; 21912.91, 24.91; 22008.96, 28.01; 22072.77, 27.70; 22136.61, 27.70; 22200.68, 30.18; 22232.83, 32.66; 22296.67, 32.66; 22360.28, 30.18; 22407.99, 28.32; 22455.67, 26.15; 22519.33, 24.29; 22583.03, 22.74; 22646.76, 21.50; 22710.74, 23.05; 22774.64, 23.67; 22838.48, 23.67; 22902.29, 23.36; 22966.10, 23.05; 23029.91, 22.74; 23093.66, 21.81; 23157.36, 20.26; 23221.20, 20.26; 23285.04, 20.26; 23348.88, 20.26; 23412.60, 19.02; 23476.44, 19.02; 23524.32, 19.02; 23604.07, 18.40; 23667.82, 17.47; 23731.57, 16.54; 23795.30, 15.30; 23859.14, 15.30; 23922.98, 15.30; 23986.79, 14.99; 24050.54, 14.06; 24114.18, 11.89; 24161.98, 10.96; 24209.66, 8.79; 24273.47, 8.48; 24321.41, 9.10], timeScale = 1/14) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Equations --------------------
    equation
// Connectors --------------------
      connect(timeTable.y, MeasHeightProfile) annotation(
        Line(points = {{12, 0}, {100, 0}}, color = {0, 0, 127}));
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {0, 0.5}, rotation = 180, extent = {{-99, 99.5}, {99, -99.5}}, fileName = "modelica://Drone/pictures/bitcoin-mine_2717142.png")}),
        Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The provided Model \"DefHeightProfile representing the topologie inspired by the Rocky Mountains.</span></span></p>

</body></html>"));
    end DefHeightProfile;

    //Model DefAir --------------------

    model DefAir
      // Model Definitions --------------------
      // Connectors --------------------
      Connectors.RealOutput airDensity annotation(
        Placement(visible = true, transformation(origin = {2, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {3, -17}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Parameters --------------------
      parameter Real airDensityVal(unit = "kg/m3") = 0.93;
      // Equations --------------------
    equation
// Equation value airDensity --------------------
      airDensity = airDensityVal;
// Annotation --------------------
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {0, 0.5}, rotation = 180, extent = {{-99, 99}, {99, -98}}, fileName = "modelica://Drone/pictures/tornados_4851771.png")}),
  Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The provided Model \"DefSensor\" representing a sensor system within a drone. This model consists of connectors, blocks, and equations to manage measured height profile data and the drone's height. </span></span></p>

<hr>
<h1><span style=\"color:#2980b9\"><strong><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameters</span></strong></span></h1>

<table>
	<thead>
		<tr>
			<th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Parameter</span></span></th>
			<th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Description</span></span></th>
			<th style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">Value</span></span></th>
		</tr>
	</thead>
	<tbody>
		<tr>
			<td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">airDensity</span></span></td>
			<td style=\"text-align:center\"><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">density of the air</font></td><td style=\"text-align:center\"><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">1.225kg/m<sup>3</sup></span></span></td>
		</tr>
	</tbody>
</table>

</body></html>"));
    end DefAir;

    // Equations --------------------
  equation
// Annotation --------------------
    annotation(
      Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(extent = {{100, -100}, {-100, 100}}, fileName = "modelica://Drone/pictures/road_12812635.png")}),
      Diagram,
      Documentation(info = "<html><head></head><body><font size=\"6\"><strong style=\"font-family: Verdana, Geneva, sans-serif; color: rgb(41, 128, 185);\">Introduction</strong><br>

</font><p><span style=\"font-size:14px\"><span style=\"font-family:Verdana,Geneva,sans-serif\">The Package \"Enviroment\" includes models for different&nbsp;</span></span><font face=\"Verdana, Geneva, sans-serif\" size=\"4\">environmental influences to the drone.&nbsp;</font></p>

</body></html>"));
  end Environment;
  annotation(
    Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Bitmap(origin = {2, 5}, rotation = 180, extent = {{-136, 101}, {137, -100}}, fileName = "modelica://Drone/pictures/top_view_drone_free.png")}),
    uses(Modelica(version = "4.0.0")),
    Documentation(info = "<html>
<img src=\"modelica://Drone/pictures/Postervorlage-01.png\" style=\"width:100.0%\"> </img>
</html>", __OpenModelica_infoHeader = "<html><head></head><body><h1><span style=\"color:#2980b9\"><span style=\"font-size:36px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"><strong>Modelica Drone Documentation</strong></span></span></span></h1>

<hr>
<p><span style=\"color:#2c3e50\"><span style=\"font-family: Verdana, Geneva, sans-serif;\"><strong><font size=\"5\">Modelica Seminar</font><span style=\"font-size: 16px;\"> </span><font size=\"5\">|&nbsp;</font></strong></span><strong><font size=\"5\">WS23/24 |</font>&nbsp;</strong><span style=\"font-family: Verdana, Geneva, sans-serif;\"><strong><font size=\"5\">MMS 1</font></strong></span></span></p>

<p><span style=\"color:#2c3e50\"><span style=\"font-size:16px\"><span style=\"font-family:Verdana,Geneva,sans-serif\"><strong>Christoph Koscheck // Alexander Leitz //&nbsp;</strong></span><span style=\"font-family:Verdana,Geneva,sans-serif\"><strong>Paul Smidt</strong></span></span></span></p>
<hr></body></html>"));
end Drone;
