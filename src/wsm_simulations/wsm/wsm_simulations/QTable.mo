within wsm_simulations;

model QTable
  parameter Integer stateDims = 3;
  Modelica.Blocks.Interfaces.RealInput learning_rate annotation(Placement(visible = true, transformation(origin = {-150, 8.366}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-10, -100}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
  Modelica.Blocks.Interfaces.RealInput epsilon annotation(Placement(visible = true, transformation(origin = {-150, -25}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {35, -100}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
  Modelica.Blocks.Interfaces.RealInput discount_factor annotation(Placement(visible = true, transformation(origin = {-150, -52.799}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {80, -100}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
  Modelica.Blocks.Interfaces.RealInput state[stateDims] annotation(Placement(visible = true, transformation(origin = {-17.522, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput reward annotation(Placement(visible = true, transformation(origin = {-150, 38.107}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  SettingConnector stateSettingConnector1[stateDims] annotation(Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -100}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  SettingConnector actionSettingConnector1 annotation(Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-50, -100}, extent = {{-11.242, -11.242}, {11.242, 11.242}}, rotation = 0)));
  discrete Modelica.Blocks.Interfaces.RealOutput action annotation(Placement(visible = true, transformation(origin = {45, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  class QTableOBJ
    extends ExternalObject;

    function constructor
      input Real learning_rate;
      input Real epsilon;
      input Real discount_factor;
      output QTableOBJ qtable;
    
      external "C" qtable = QTableConstructor(learning_rate, epsilon, discount_factor) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
    end constructor;

    function destructor
      input QTableOBJ qtable;
    
      external "C" QTableDestructor(qtable) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
    end destructor;
  end QTableOBJ;

  function getAction
    input QTableOBJ qtable;
    input Integer stateDims;
    input Modelica.Blocks.Interfaces.RealInput state[stateDims];
    input Real reward;
    input Real learning_rate;
    input Real epsilon;
    input Real discount_factor;
    output Real action;
  
    external "C" action = choose_action(qtable, state, stateDims, reward, learning_rate, epsilon, discount_factor) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
  end getAction;

  QTableOBJ qtable = QTableOBJ.constructor(learning_rate, epsilon, discount_factor);
  Modelica.Blocks.Interfaces.BooleanInput trigger annotation(Placement(visible = true, transformation(origin = {-150, 63.458}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  when {trigger} then
    action = getAction(qtable, stateDims, state, reward, learning_rate, epsilon, discount_factor);
  end when;
  annotation(Diagram(coordinateSystem(extent = {{-150, -90}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {190, -60}, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name", horizontalAlignment = TextAlignment.Left), Text(visible = true, origin = {0, -0.617}, textColor = {0, 128, 128}, extent = {{-103.149, -80.617}, {103.149, 80.617}}, textString = "Q", textStyle = {TextStyle.Bold}), Ellipse(visible = true, origin = {-50, -100}, lineColor = {128, 0, 0}, fillPattern = FillPattern.Sphere, extent = {{-17.33, -17.33}, {17.33, 17.33}})}));
end QTable;
