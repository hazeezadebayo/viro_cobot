

console.log("Script started running.");
        // =============================
        // =============================
        const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });       // <== network-mode:host | start host TS
        // const ros = new ROSLIB.Ros({ url: "ws://100.64.69.46:9090" }); // <== TS docker ip | netmode:host | stop host TS <=OR= TS docker ip | no netmode | start host TS
        // --------
        // When the Rosbridge server connects, fill the span with id "status" with "successful"
        ros.on("connection", () => {
                document.getElementById("status").innerHTML = "successful";
                console.log('Connected to ROS Bridge.');
                });
        // --------
        // When the Rosbridge server experiences an error, fill the "status" span with the returned error
        ros.on("error", (error) => {
                document.getElementById("status").innerHTML = `errored out (${error})`;
                console.error('Error connecting to ROS Bridge:', error);
                });
        // --------
        // When the Rosbridge server shuts down, fill the "status" span with "closed"
        ros.on("close", () => {
                document.getElementById("status").innerHTML = "closed";
                console.log('Disconnected from ROS Bridge.');
                });
        // =============================
        // =============================
        const my_topic = new ROSLIB.Topic({
                ros: ros,
                name: "/gui_command",
                messageType: "std_msgs/msg/String",
        });
        my_topic.subscribe((message) => {
                const messageData = message.data.split(',');
                const messageType = messageData[0];
                const messageValue = messageData[1];
                const messageElement = document.getElementById("messages");
                if (messageElement) {
                    messageElement.innerHTML = `Type: ${messageType}, Value: ${messageValue}`;
                }
         });
        // --------
        const gui_joint_states_topic = new ROSLIB.Topic({
                ros: ros,
                name: "/gui_joint_states",
                messageType: "std_msgs/msg/String",
        });
        // Subscribe to the GUI joint states topic
        gui_joint_states_topic.subscribe((message) => {
                const jointValues = message.data.split(',');
                const roundedJointValues = jointValues.map(value => parseFloat(value).toFixed(4));
                const jointElement = document.getElementById("joint_values");
                if (jointElement) {
                    jointElement.innerHTML = `Joint Values: ${roundedJointValues.join(', ')}`;
                }
        });
        // --------
        function sendMessage(message) {
                const key = Object.keys(message)[0];
                const value = message[key];
                const data_str = key + "," + value;
                
                var msg = new ROSLIB.Message({
                    data: data_str
                });
                
                my_topic.publish(msg);
                console.log('Published message:', data_str);
                
                // Feedback for user
                const msgEl = document.getElementById("messages");
                if (msgEl) msgEl.innerHTML = data_str;
        }
        
        function sendSpeed(message) {
                var msg = new ROSLIB.Message({
                    data: 'speed_val' + ',' + message['speed_val']
                });
                const speedEl = document.getElementById('speedValue');
                if (speedEl) speedEl.textContent = message['speed_val'];
                my_topic.publish(msg);
                console.log('Published message:', msg.data);
            }

        function sendManualWrench(zeros = false) {
            let fx = document.getElementById('fext_x').value;
            let fy = document.getElementById('fext_y').value;
            let fz = document.getElementById('fext_z').value;
            let tx = document.getElementById('fext_tx').value;
            let ty = document.getElementById('fext_ty').value;
            let tz = document.getElementById('fext_tz').value;
            
            if (zeros) {
                fx = fy = fz = tx = ty = tz = 0;
            }

            const data_str = `manual_force,${fx},${fy},${fz},${tx},${ty},${tz}`;
            sendMessage_raw(data_str);
        }

        function sendMessage_raw(data_str) {
            var msg = new ROSLIB.Message({ data: data_str });
            my_topic.publish(msg);
            console.log('Published raw:', data_str);
            const msgEl = document.getElementById("messages");
            if (msgEl) msgEl.innerHTML = data_str;
        }
        // =============================
        // =============================
