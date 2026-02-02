<?php
header("Content-Type: application/json; charset=utf-8");

$host = "127.0.0.1";     // MySQL server (same machine)
$user = "capstone_user";     // your MySQL user
$pass = "LetsGetCoding!"; // your MySQL password
$db   = "WebsiteDemo";   // your database name

$conn = new mysqli($host, $user, $pass, $db);
if ($conn->connect_error) {
  http_response_code(500);
  echo json_encode(["error" => "db_connect_failed"]);
  exit;
}

$sql = "SELECT name, weightWarning, weightError, brakeWarning, brakeError, gas
        FROM drivers
        ORDER BY name ASC";

$res = $conn->query($sql);
if (!$res) {
  http_response_code(500);
  echo json_encode(["error" => "query_failed"]);
  exit;
}

$out = [];
while ($row = $res->fetch_assoc()) {
  $row["weightWarning"] = (int)$row["weightWarning"];
  $row["weightError"]   = (int)$row["weightError"];
  $row["brakeWarning"]  = (int)$row["brakeWarning"];
  $row["brakeError"]    = (int)$row["brakeError"];
  $row["gas"]           = (int)$row["gas"];
  $out[] = $row;
}

echo json_encode($out);

