// ----------------------
// DRIVER CLASS
// ----------------------
class Driver {
  constructor(name, CW, MW, P, LH) {
    this.name = name;
    this.maxWeight = MW;
    this.currentWeight = CW;
    this.pitch = P;
    this.loadHeight = LH;
  }
}

// Only ONE driver for now
let driver = new Driver("Matt", 0, 0, 0, 0);

const driverList = document.getElementById("driverList");


// ----------------------
// BUILD UI FOR SINGLE DRIVER
// ----------------------
const card = document.createElement("div");
card.className = "driver-card bg-white p-4 rounded-xl shadow cursor-pointer";

card.innerHTML = `
  <div class="flex justify-between items-center">
    <div>
      <h2 class="text-xl font-semibold text-gray-800">${driver.name}</h2>
    </div>
    <span class="text-gray-600">▼</span>
  </div>

  <div id="details" class="hidden mt-4 border-t pt-4">
    <div class="grid grid-cols-2 gap-4 mb-4">
      <div class="bg-gray-50 p-3 rounded-lg text-center">
        <p class="text-sm text-gray-500">Max Weight</p>
        <p class="text-xl font-bold text-green-600" id="maxWeight">${driver.maxWeight}</p>
      </div>
      <div class="bg-gray-50 p-3 rounded-lg text-center">
        <p class="text-sm text-gray-500">Current Weight</p>
        <p class="text-xl font-bold text-blue-600" id="currentWeight">${driver.currentWeight}</p>
      </div>
    </div>
  </div>
`;

driverList.appendChild(card);


// Expand/collapse card
card.addEventListener("click", () => {
  const details = document.getElementById("details");
  const icon = card.querySelector("span");
  const isHidden = details.classList.contains("hidden");

  if (isHidden) {
    details.classList.remove("hidden");
    icon.textContent = "▲";
  } else {
    details.classList.add("hidden");
    icon.textContent = "▼";
  }
});



// ----------------------
// MQTT CONNECT
// ----------------------
const clientID = "webclient_" + Math.floor(Math.random() * 10000);
const client = new Paho.MQTT.Client("YOUR_MQTT_IP", 9001, clientID);

client.onMessageArrived = onMessageReceived;

client.connect({
  onSuccess: () => {
    console.log("Connected to MQTT");
    client.subscribe("forkLift1/sensors");
  },
  useSSL: false
});



// ----------------------
// HANDLE INCOMING MQTT MESSAGE
// ----------------------
function onMessageReceived(message) {
  const payload = message.payloadString;
  console.log("MQTT:", payload);

  // Expecting format:
  // Driver:Matt,MaxWeight:123.4,CurrentWeight:55.5,Pitch:10,LoadHeight:200

  const parts = payload.split(",");
  let data = {};

  parts.forEach(p => {
    let [k, v] = p.split(":");
    data[k.trim()] = v.trim();
  });

  // Update driver object
  driver.maxWeight = parseFloat(data["MaxWeight"]);
  driver.currentWeight = parseFloat(data["CurrentWeight"]);
  driver.pitch = parseInt(data["Pitch"]);
  driver.loadHeight = parseInt(data["LoadHeight"]);

  updateUI();
}



// ----------------------
// UPDATE UI VALUES
// ----------------------
function updateUI() {
  document.getElementById("maxWeight").textContent = driver.maxWeight;
  document.getElementById("currentWeight").textContent = driver.currentWeight;
}
