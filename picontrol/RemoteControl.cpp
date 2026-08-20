// 08-20-2026: This file is unused, see dac_daemon.py instead.

//******************************************************************************************************************************************************************************************/
// Remote control of pendulum experiment via MQTT messages, with status light feedback and Simulink program dispatching */
//******************************************************************************************************************************************************************************************/

#include <iostream>
#include <cstring>
#include <mosquitto.h>

#include <gpiod.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "json.hpp"
using json = nlohmann::json;
json json_data;

json pgm_dispatch = json::parse(R"(
{
	"CartControl" : "CartControlPiMod2.elf",
	"TestCartPend1" : "TestCartPend1PiMod2.elf",
	"CartIdent" : "CartIdentPiMod2.elf",
	"CraneIdent" : "CraneIdentPiMod2.elf",
	"CraneStab" : "CraneStabPiMod2.elf",
	"InvPendIdent" : "InvPendIdentPiMod2.elf",
	"PendstabPD" : "PendStabPDPiMod2.elf",
	"PendSwingUp" : "PendSwingUpPiMod2.elf",
	"PendulumFriction" : "PendulumFrictionPiMod2.elf",
	"PendulumTest" : "PendulumTestPiMod2.elf",
	"SwingHoldPendulum" : "SwingHoldPendulumPiMod2.elf",
	"SwingHoldPendulumExtra" : "SwingHoldPendulumExtraPiMod2.elf",
	"UpDownPendulum" : "UpDownPendulumPiMod2.elf"
}
)");
	
std::string expState = "initialized";
std::string sim_pgm = "";

//******************************************************************************************************************************************************************************************/
//* Simulink Functions */
//******************************************************************************************************************************************************************************************/
//* Setup Simulink */
void setupSimulink() {
	std::cout << "Simulink Setup - Started" << std::endl;
	std::cout << "               - Complete" << std::endl;
	return;
} 

static void process_parameters(std::string exp,json parameters) {
	
	json parm_dispatch = json::parse(R"(
	{
		"CartControl" : 
			{
				"PID_D1_derivative":"PID1DD",
				"PID_D1_integral" : "PID1DI",
				"PID_D1_proportional" : "PID1DP",
				"PID_D_derivative" : "PIDDD",
				"PID_D_integral" : "PIDDI",
				"PID_D_proportional" : "PIDDP"
			},
		"TestCartPend1" :
			{
				"kd" : "kd",
				"ki" : "ki",
				"kp" : "kp"
			},
		"CartIdent" :
			{
			},
		"CraneStab" :
			{
				"PID_D1_derivative":"PID1DD",
				"PID_D1_integral" : "PID1DI",
				"PID_D1_proportional" : "PID1DP",
				"PID_D_derivative" : "PIDDD",
				"PID_D_integral" : "PIDDI",
				"PID_D_proportional" : "PIDDP"
			},
		"InvPendIdent" :
			{
				"PID_1_derivative":"PID1D",
				"PID_1_integral" : "PID1I",
				"PID_1_proportional" : "PID1P",
				"PID_D1_derivative" : "PID1DD",
				"PID_D1_integral" : "PID1DI",
				"PID_D1_proportional" : "PID1DP",
				"PID_D_derivative" : "PIDDD",
				"PID_D_integral"  : "PIDDI",
				"PID_D_proportional" : "PIDDP",
				"PID_derivative" : "PIDD",
				"PID_integral" : "PIDI",
				"PID_proportional" : "PIDP"
			},
		"PendstabPD" :
			{
				"PID_1_derivative":"PID1D",
				"PID_1_integral" : "PID1I",
				"PID_1_proportional" : "PID1P",
				"PID_D1_derivative" : "PID1DD",
				"PID_D1_integral" : "PID1DI",
				"PID_D1_proportional" : "PID1DP"
			},
		"PendSwingUp" : 
			{
				"angle comparison value":"ACV",
				"voltage amplitude" : "VA"
			},
		"PendulumFriction" : 
			{
			},
		"PendulumTest" :
			{
			},
		"SwingHoldPendulum" : 
			{
				"cart_friction":"CF",
				"initial_angle" : "IA",
				"moment_inertia" : "MI",
				"pendulum_damping" : "PD"
			},
		"SwingHoldPendulumExtra" : 
			{
				"PID_D1_derivative" : "PID1DD",
				"PID_D1_integral" : "PID1DI",
				"PID_D1_proportional" : "PID1DP",
				"PID_D_derivative" : "PIDDD",
				"PID_D_integral"  : "PIDDI",
				"PID_D_proportional" : "PIDDP"
			},
		"UpDownPendulum" :
			{
				"PID_DIPS_derivative" : "IPSPIDDD",
				"PID_DIPS_integral" : "IPSPIDDI",
				"PID_DIPS_proportional" : "IPSPID1DP",
				"PID_D1IPS_derivative" : "IPSPID1DD",
				"PID_D1IPS_integral" : "IPSPID1DI",
				"PID_D1IPS_proportional" : "IPSPID1DP",
				"PID_DCC_derivative" : "CCPIDDD",
				"PID_DCC_integral" : "CCPIDDI",
				"PID_DCC_proportional" : "CCPIDDP",
				"PID_D1CC_derivative" : "CCPID1DD",
				"PID_D1CC_integral" : "CCPID1DI",
				"PID_D1CC_proportional" : "CCPID1DP"
			}
		
		}
	)");	                                 
	std::cout << "Processing experiment " << exp << std::endl;                                 
	std::cout << "Processing parameters " << parameters.dump() << std::endl;
	
	json expParms = parm_dispatch[exp];
	
	json data = json::parse(parameters.dump());
	for (const auto& [key, value] : data.items()) {
		std::string fullKey = key;
		std::string modelKey = expParms[key];
		// Print primitive values (string, int, bool, null)
		std::cout << fullKey << " = " << value << "-->" << modelKey << "\n";

	}
	
	return;
}

static void dispatch_to_simulink(std::string exp, json parameters) {
	const std::string library = "/home/owlsley/picontrol/MATLAB_ws/R2025b";
	sim_pgm = pgm_dispatch[exp];
	
	std::cout << "Dispatching to program " << sim_pgm << std::endl;
			
	process_parameters(exp, parameters);
	return;
	
	std::cout << "Started Simulink program " << sim_pgm << std::endl;
	int retval = system(("sudo " + library + "/" + sim_pgm + " &").c_str());
	if (retval == -1) {
		std::cerr << "Failed to execute Simulink program." << std::endl;
	}
}

static void kill_simulink() {
	std::cout << "Killing Simulink program " << sim_pgm << std::endl;
	int retval = system(("ps -ef | grep " + sim_pgm + " | awk '{print $2}' | xargs kill &").c_str());
	if (retval == -1) {
		std::cerr << "Failed to kill Simulink program." << std::endl;
	}
}
//******************************************************************************************************************************************************************************************/
//* MQTT Functions */
//******************************************************************************************************************************************************************************************/
const char *host = "sciencelabtoyou.com"; // Change to your broker address
int port = 1885;
const char *client_id = "cpp_subscriber";
struct mosquitto *mosq;

//*************************/
// MQTT message processing
//*************************/
void process_message(std::string payload) {

	json_data = json::parse(payload);
	
	auto command = json_data.find("command");
	std::cout << "Received command: " << *command << std::endl;
	
	auto experiment = json_data.find("experiment");
	std::cout << "Experiment: " << *experiment << std::endl;	
		
	if (*command == "sta") {
		auto parameters = json_data.find("parameters");
		std::cout << "Parameters: " << *parameters << std::endl;
		dispatch_to_simulink(*experiment, *parameters);
	}
	else if (*command == "sto") {
		kill_simulink();
	}
	else {
		printf("Unknown command: %s\n", payload);
	}
}

// Callback when the client connects to the broker
void on_connect(struct mosquitto *mosq, void *userdata, int rc) {
	if (rc == 0) {
		std::cout << "Connected to broker successfully." << std::endl;
		// Subscribe to a topic
		const char* topic = "pendulum/cmd";
		int ret = mosquitto_subscribe(mosq, nullptr, topic, 0);
		if (ret != MOSQ_ERR_SUCCESS) {
			std::cerr << "Failed to subscribe: " << mosquitto_strerror(ret) << std::endl;
		}
		else {
			std::cout << "Subscribed to topic " << topic << std::endl;
		}
	}
	else {
		std::cerr << "Connection failed: " << mosquitto_strerror(rc) << std::endl;
	}
}

// Callback when a message is received
void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg) {
	std::cout << "Message received on topic '" << msg->topic << "': "
	          << (msg->payloadlen ? (char*)msg->payload : "(empty)") << std::endl;
	std::string message;
	message = (char*)msg->payload;	
	process_message(message);
}

// Callback when the client disconnects
void on_disconnect(struct mosquitto *mosq, void *userdata, int rc) {
	std::cout << "Disconnected from broker." << std::endl;
}

//* Setup MQTT */
int setupMQTT() {
	std::cout << "MQTT Setup - Started" << std::endl;

	// Initialize the Mosquitto library
	mosquitto_lib_init();

	// Create a new Mosquitto client instance
	mosq = mosquitto_new(client_id, true, nullptr);
	if (!mosq) {
		std::cerr << "Failed to create Mosquitto instance." << std::endl;
		mosquitto_lib_cleanup();
		return 1;
	}

	// Set callbacks
	mosquitto_connect_callback_set(mosq, on_connect);
	mosquitto_message_callback_set(mosq, on_message);
	mosquitto_disconnect_callback_set(mosq, on_disconnect);

	// Connect to the broker
	int ret = mosquitto_connect(mosq, host, port, 60);
	if (ret != MOSQ_ERR_SUCCESS) {
		std::cerr << "Unable to connect: " << mosquitto_strerror(ret) << std::endl;
		mosquitto_destroy(mosq);
		mosquitto_lib_cleanup();
		return 1;
	}
	
	std::cout << "           - Complete" << std::endl;
	return 0;
}

//* Cleanup MQTT */
int cleanupMQTT() {
	
	std::cout << "MQTT Cleanup - Started" << std::endl;
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
	std::cout << "             - Complete" << std::endl;
	return 0;
}
//******************************************************************************************************************************************************************************************/
//* ADC Functions */
//******************************************************************************************************************************************************************************************/
//* Setup DAC */
void setupADC() {
	std::cout << "DAC Setup - Started" << std::endl;
	std::cout << "          - Complete" << std::endl;
	return;
}

//******************************************************************************************************************************************************************************************/
//* I2C Functions */
//******************************************************************************************************************************************************************************************/
//* Setup I2C */
void setupI2C() {
	std::cout << "I2C Setup - Started" << std::endl;
	std::cout << "          - Complete" << std::endl;
	return;
}

//******************************************************************************************************************************************************************************************/
//* GPIO Functions */
//******************************************************************************************************************************************************************************************/
const char *chipname = "gpiochip0";
struct gpiod_chip *chip;
const char *gpioHomeStartName = "GPIO16";
struct gpiod_line *gpioHomeStart;
const char *gpioStopName = "GPIO20";
struct gpiod_line *gpioStop;
const char *gpioHomeDoneName = "GPIO13";
struct gpiod_line *gpioHomeDone;

//* Setup GPIO */
void setupGPIO() {
	std::cout << "GPIO Setup - Started" << std::endl;

	chip = gpiod_chip_open_by_name(chipname);
	if (!chip) {
		perror("Open chip failed\n");
		return;
	}
	
	//* Setup GPIO Home Start line */
	gpioHomeStart = gpiod_chip_find_line(chip, gpioHomeStartName);
	if (!gpioHomeStart) {
		perror("GPIO Home Start failed\n");
		gpiod_chip_close(chip);
		return;
	}
	if (gpiod_line_request_output(gpioHomeStart, "gpio_test", 0) < 0) {
		perror("GPIO Home Start as output failed\n");
		gpiod_chip_close(chip);
		return;
	}
	gpiod_line_set_value(gpioHomeStart, 0);		// Off
	
	//* Setup GPIO Stop line */
	gpioStop = gpiod_chip_find_line(chip, gpioStopName);
	if (!gpioStop) {
		perror("GPIO Stop failed\n");
		gpiod_chip_close(chip);
		return;
	}
	if (gpiod_line_request_output(gpioStop, "gpio_test", 0) < 0) {
		perror("GPIO Stop as output failed\n");
		gpiod_chip_close(chip);
		return;
	}
	gpiod_line_set_value(gpioStop, 0);		// Off
		
	//* Setup GPIO Home Done line */
	gpioHomeDone = gpiod_chip_find_line(chip, gpioHomeDoneName);
	if (!gpioHomeDone) {
		perror("GPIO Home Done failed\n");
		gpiod_chip_close(chip);
		return;
	}
	if (gpiod_line_request_input(gpioHomeDone, "gpio_test") < 0) {
		perror("GPIO Home Done as input failed\n");
		gpiod_chip_close(chip);
		return;
	}
	gpiod_line_set_value(gpioHomeDone, 0);		// Off
	
	std::cout << "           - Complete" << std::endl;
	return;
}
//******************************************************************************************************************************************************************************************/
//* Main Function */
//******************************************************************************************************************************************************************************************/
int main() {

	setupADC();
	setupI2C();
	setupGPIO();
	setupMQTT();
	setupSimulink();
	
	while (true) {
		// Main loop can be used for other tasks if needed
		mosquitto_loop_forever(mosq, -1, 1);		
	}
	
	cleanupMQTT();
	
	return 0;
}
