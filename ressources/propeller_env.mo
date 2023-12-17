package propeller_env 
   class propeller
    //Parameter
    parameter Real c_w = 0.5 "Drag coefficient";
    parameter Modelica.Units.SI.Diameter d = 0.3 "Diameter";
    parameter Real n_o_b = 2 "Number of blades";
    parameter Modelica.Units.SI.Area A_p = 0.13 "Area of the propeller";
    annotation(
      Icon(graphics = {Line(origin = {6.05, -26.45}, points = {{-15, 24}, {-15, -24}, {15, -24}, {15, 22}, {-15, 22}, {-1, 22}}, thickness = 0.5), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {56, -6}, extent = {{36, -6}, {-36, 6}})}));
  end propeller;

  model Auftrieb
    //Parameter
    parameter Real p_air = 1.225 "Air density";
    parameter Real A_prop = 0.13 "Area of the propeller";
    parameter Real r_prop = 0.2 "Radius of the propeller";
    parameter Real d_prop = 0.4 "Diameter of the propeller";
    parameter Real C_l = 0.006 "Lift coefficient";
    parameter Real NoP = 4 "Number of propellers";
    parameter Real initialSpeed = 1000 "Start RPM";
    parameter Real finalSpeed = 3000 "End RPM";
    parameter Real Masse = 4 "System Mass";
  
    //Variablen
    Real n_prop "Propeller speed";
    Real w_prop "Propeller angular velocity";
    Real F_auf "Lift force";
    Real F_prop "Propeller force";
    Real T_prop "Propeller torque";
    Real Acc_prop "acceleration per propeller";
    Real V_prop "Velocity of System";
    
    equation

    n_prop = if time <= 10 then initialSpeed + (finalSpeed - initialSpeed) * time / 10 else finalSpeed;
  
    F_auf = NoP * F_prop; //Force of all propellers
    F_prop = T_prop / r_prop; //Force of one propeller
    T_prop = 0.5 * p_air * A_prop * C_l * d_prop^2 * w_prop^2; //Torque of one propeller
    w_prop = 2 * n_prop * Modelica.Constants.pi / 60; //Angular velocity of one propeller
    Acc_prop = F_prop/Masse; //Acceleration of one propeller
  der(Acc_prop) = V_prop;//Acceleration of the system
  end Auftrieb;
  
  model Auftrieb2
    //Parameter
    parameter Real p_air = 1.225 "Air density";
    parameter Real A_prop = 0.13 "Area of the propeller";
    parameter Real r_prop = 0.2 "Radius of the propeller";
    parameter Real d_prop = 0.4 "Diameter of the propeller";
    parameter Real C_l = 0.006 "Lift coefficient";
    parameter Real NoP = 4 "Number of propellers";
    parameter Real initialSpeed = 1000 "Start RPM";
    parameter Real finalSpeed = 14000 "End RPM";
    parameter Real Masse = 4 "System Mass";
    parameter Real c_w = 0.14;
    parameter Real A_drone = 0.5 "Dem wind ausgesetzte Drohnenoberfläche";
  
    //Variablen
    Real n_prop "Propeller speed";
    Real w_prop "Propeller angular velocity";
    Real F_auf "Lift force";
    Real F_prop "Propeller force";
    Real T_prop "Propeller torque";
    Real Acc_prop "acceleration per propeller";
    Real V_prop "Velocity of System";
    Real F_w "Flow resistance";
    equation
  
    n_prop = if time <= 10 then initialSpeed + (finalSpeed - initialSpeed) * time / 10 else finalSpeed;
  
    F_auf = NoP * F_prop; //Force of all propellers
    F_prop = T_prop / r_prop - F_w; //Force of one propeller
    T_prop = 0.5 * p_air * A_prop * C_l * d_prop^2 * w_prop^2; //Torque of one propeller
    w_prop = d_prop * n_prop * Modelica.Constants.pi / 60; //Angular velocity of one propeller
    Acc_prop = F_prop/Masse; //Acceleration of one propeller
    der(V_prop) = Acc_prop; //Acceleration of the system
    F_w = A_drone * c_w * 0.5 * p_air *V_prop^2; //Flow resistance
  
  
  end Auftrieb2;
end propeller_env;
