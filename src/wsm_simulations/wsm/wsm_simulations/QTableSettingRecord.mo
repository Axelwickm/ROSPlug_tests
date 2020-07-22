within wsm_simulations;

record QTableSettingRecord
  Real minimum;
  Real maximum;
  Integer count;
  annotation(Diagram(coordinateSystem(extent = {{-150, -90}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 60}, {150, 100}}, textString = "%name"), Rectangle(visible = true, origin = {0, -25}, lineColor = {0, 114, 195}, fillColor = {128, 202, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -75}, {100, 75}}, radius = 10), Line(visible = true, points = {{-100, 0}, {100, 0}}, color = {0, 127, 255}), Line(visible = true, origin = {0, -50}, points = {{-100, 0}, {100, 0}}, color = {0, 127, 255}), Line(visible = true, origin = {0, -25}, points = {{0, 75}, {0, -75}}, color = {0, 127, 255})}));
end QTableSettingRecord;
