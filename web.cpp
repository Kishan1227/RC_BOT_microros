#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "state.h"
#include "encoders.h"
#include "motors.h"
#include "autonomous.h"
#include "web.h"

extern WebServer server;

const char htmlPage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{font-family:sans-serif;text-align:center;background:#111;color:#eee}
.btn{width:90px;height:90px;font-size:18px;margin:6px;border-radius:12px;border:none;background:#333;color:#fff;touch-action:none}
.btn:active{background:#0a84ff}
#pad{display:grid;grid-template-columns:100px 100px 100px;justify-content:center}
input{width:130px;padding:10px;margin:5px;font-size:18px;text-align:center}
.action{width:190px;height:50px;font-size:18px;margin:8px;border-radius:10px;border:none}
.section{margin-top:25px}
#stats{margin-top:20px;font-size:15px;line-height:1.7}
</style>
</head>
<body>
<h2>ESP32 Robot Control</h2>
<div id="pad">
<div></div><button class="btn" id="F">FWD</button><div></div>
<button class="btn" id="L">LEFT</button><button class="btn" id="S">STOP</button><button class="btn" id="R">RIGHT</button>
<div></div><button class="btn" id="B">BACK</button><div></div>
</div>

<div class="section">
<h3>Turn</h3>
<input type="text" id="turnRight" value="" min="0" placeholder="Right turn (degrees)">
<input type="text" id="turnLeft" value="" min="0" placeholder="Left turn (degrees)"><br>
<button class="action" onclick="turnRobot()">TURN</button>
</div>

<div class="section">
<h3>X / Y Coordinate</h3>
<input type="text" id="xForward" value="" min="0" placeholder="X Forward (mm)">
<input type="text" id="xBackward" value="" min="0" placeholder="X Backward (mm)"><br>
<input type="text" id="yRight" value="" min="0" placeholder="Y Right (mm)">
<input type="text" id="yLeft" value="" min="0" placeholder="Y Left (mm)"><br>
<input type="text" id="zAngle" value="" placeholder="Final Angle (degrees)"><br>
<button class="action" onclick="moveXY()">MOVE X / Y / Z</button>
</div>

<div><button class="action" onclick="stopRobot()">STOP ROBOT</button></div>
<div id="stats">Loading...</div>

<script>
function send(dir,state){fetch('/cmd?d='+dir+'&s='+state)}

let manualTimer = null;
let currentManualCommand = null;

function startManual(dir) {

  // Stop any previous repeating command
  if (manualTimer !== null) {
    clearInterval(manualTimer);
  }

  currentManualCommand = dir;

  // Send immediately
  send(dir, 1);

  // Keep command alive while button is held
  manualTimer = setInterval(() => {
    send(dir, 1);
  }, 300);
}

function stopManual() {

  // Stop repeating command
  if (manualTimer !== null) {
    clearInterval(manualTimer);
    manualTimer = null;
  }

  // Tell ESP32 to stop this command
  if (currentManualCommand !== null) {
    send(currentManualCommand, 0);
    currentManualCommand = null;
  }
}

function turnRobot(){
  let right=parseFloat(document.getElementById('turnRight').value)||0;
  let left=parseFloat(document.getElementById('turnLeft').value)||0;
  if(right>0&&left>0){alert('Enter either Right turn or Left turn, not both.');return}
  let angle=0;
  if(right>0) angle=right; else if(left>0) angle=-left;
  if(angle==0){alert('Enter a turn angle.');return}
  fetch('/turn?angle='+angle);
}

function moveXY(){
  let xForward=parseFloat(document.getElementById('xForward').value)||0;
  let xBackward=parseFloat(document.getElementById('xBackward').value)||0;
  let yRight=parseFloat(document.getElementById('yRight').value)||0;
  let yLeft=parseFloat(document.getElementById('yLeft').value)||0;
  let z=parseFloat(document.getElementById('zAngle').value)||0;
  if(xForward>0&&xBackward>0){alert('Enter either X Forward or X Backward, not both.');return}
  if(yRight>0&&yLeft>0){alert('Enter either Y Right or Y Left, not both.');return}
  let x=0,y=0;
  if(xForward>0)x=xForward;else if(xBackward>0)x=-xBackward;
  if(yRight>0)y=yRight;else if(yLeft>0)y=-yLeft;
  if(x==0&&y==0){alert('Enter a distance in at least one X/Y direction.');return}
  fetch('/xy?x='+x+'&y='+y+'&z='+z);
}

function stopRobot(){fetch('/stop')}

['F','B','L','R'].forEach(id => {

  const el = document.getElementById(id);

  // Mouse press
  el.addEventListener('mousedown', e => {
      e.preventDefault();
      startManual(id);
    }
  );

  // Mouse release
  el.addEventListener('mouseup',e => {
      e.preventDefault();
      stopManual();
    });

  // Mouse leaves button
  el.addEventListener('mouseleave',e => {
      e.preventDefault();
      stopManual();
    });

  // Touch press
  el.addEventListener('touchstart',e => {
      e.preventDefault();
      startManual(id);
    });

  // Touch release
  el.addEventListener('touchend',e => {
      e.preventDefault();
      stopManual();
    });

  // Touch cancelled
  el.addEventListener('touchcancel',e => {
      e.preventDefault();
      stopManual();
    });
});

setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('stats').innerHTML=
      'L ticks: '+d.lt+' &nbsp; R ticks: '+d.rt+'<br>'+
      'L dist: '+d.ld+' mm &nbsp; R dist: '+d.rd+' mm<br>'+
      'Distance: '+d.avg+' mm<br>'+
      'Heading: '+d.heading+'&deg;<br>'+
      'Mode: '+d.auto;
  });
},300);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", htmlPage);
}

void handleCmd() {
  autoState = AUTO_IDLE;
  String d = server.arg("d");
  bool state = server.arg("s") == "1";

  if (d == "F") fwd = state;
  else if (d == "B") bwd = state;
  else if (d == "L") left = state;
  else if (d == "R") right = state;

  lastCmdTime = millis();
  server.send(200, "text/plain", "ok");
}

void handleTurn() {
  float angle = server.arg("angle").toFloat();
  fwd = bwd = left = right = false;
  startTurn(angle);
  server.send(200, "text/plain", "turn started");
}

void handleXY() {
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  requestedZ = server.arg("z").toFloat();
  fwd = bwd = left = right = false;
  startXY(x, y);
  server.send(200, "text/plain", "xy started");
}

void handleStop() {
  fwd = bwd = left = right = false;
  autoState = AUTO_IDLE;
  setMotor(0, 0);
  resetPID();
  server.send(200, "text/plain", "stopped");
}

void handleStatus() {
  float distL = getLeftDistance();
  float distR = getRightDistance();
  float distAvg = (distL + distR) / 2.0f;
  float heading = getHeadingDeg();

  String autoName;
  if (autoState == AUTO_IDLE) autoName = "IDLE";
  else if (autoState == AUTO_TURN) autoName = "TURN";
  else if (autoState == AUTO_MOVE_X) autoName = "X";
  else if (autoState == AUTO_TURN_Y) autoName = "TURN Y";
  else if (autoState == AUTO_MOVE_Y) autoName = "Y";
  else if (autoState == AUTO_TURN_TO_GOAL) autoName = "TURN TO GOAL";
  else if (autoState == AUTO_MOVE_GOAL) autoName = "MOVE TO GOAL";
  else if (autoState == AUTO_FINAL_TURN) autoName = "FINAL TURN";
  else autoName = "DONE";

  String json = "{";
  json += "\"lt\":" + String(leftTicks) + ",";
  json += "\"rt\":" + String(rightTicks) + ",";
  json += "\"ld\":" + String(distL,1) + ",";
  json += "\"rd\":" + String(distR,1) + ",";
  json += "\"avg\":" + String(distAvg,1) + ",";
  json += "\"heading\":" + String(heading,1) + ",";
  json += "\"auto\":\"" + autoName + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/turn", handleTurn);
  server.on("/xy", handleXY);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);
  server.begin();
}

void handleWebClient() {
  server.handleClient();
}
