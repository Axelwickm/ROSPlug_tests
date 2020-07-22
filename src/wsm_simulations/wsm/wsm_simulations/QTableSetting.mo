within wsm_simulations;

model QTableSetting "State or action minimum and maximum value, and number of values"
  parameter Real minimum = 0 "Lowest real value";
  parameter Real maximum = 1 "Highest real value";
  parameter Integer count = 10 "Number of steps between these above";
  SettingConnector settingConnector1 annotation(Placement(visible = true, transformation(origin = {130.573, -9}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {78.376, -23.524}, extent = {{-16.476, -16.476}, {16.476, 16.476}}, rotation = 0)));
equation
  settingConnector1.r.minimum = minimum;
  settingConnector1.r.maximum = maximum;
  settingConnector1.r.count = count;
  annotation(Diagram(coordinateSystem(extent = {{-150, -90}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, lineColor = {56, 56, 56}, fillColor = {246, 246, 246}, fillPattern = FillPattern.Solid, points = {{-80, -100}, {-80, 100}, {20, 100}, {20, 40}, {80, 40}, {80, -100}, {-80, -100}}), Polygon(visible = true, lineColor = {56, 56, 56}, fillColor = {230, 230, 230}, fillPattern = FillPattern.Solid, points = {{20, 100}, {80, 40}, {20, 40}, {20, 100}}), Line(visible = true, origin = {0, -17.5}, points = {{2, -12}, {50, -12}}, color = {56, 56, 56}), Line(visible = true, origin = {0, -2.5}, points = {{2, -60}, {50, -60}}, color = {56, 56, 56}), Ellipse(visible = true, origin = {-35, -62.5}, lineColor = {56, 56, 56}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid, extent = {{-12.5, -12.5}, {12.5, 12.5}}), Ellipse(visible = true, origin = {-35, -30}, lineColor = {56, 56, 56}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid, extent = {{-12.5, -12.5}, {12.5, 12.5}}), Text(visible = true, origin = {-40, 59.882}, textColor = {128, 128, 128}, extent = {{-46.368, -29.882}, {46.368, 29.882}}, textString = "Q", textStyle = {TextStyle.Bold}), Line(visible = true, origin = {0, 15}, points = {{2, -12}, {50, -12}}, color = {56, 56, 56}), Ellipse(visible = true, origin = {-35, 2.5}, lineColor = {56, 56, 56}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid, extent = {{-12.5, -12.5}, {12.5, 12.5}}), Text(visible = true, origin = {-2.448, 134.321}, extent = {{-72.448, -15.679}, {72.448, 15.679}}, textString = "%name")}));
end QTableSetting;
