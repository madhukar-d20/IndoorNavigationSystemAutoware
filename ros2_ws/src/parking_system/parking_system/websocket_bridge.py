import websockets
import json
import asyncio
import requests 

async def listen_for_tasks():
    #Cloud WebSocket URL
    ws_url = "wss://firewire-hospital-therefore-acre.trycloudflare.com/ws/machine"

    #Local Navigation Endpoint
    local_nav_url = "http://localhost:8081/navigate"

    print(f"Connecting to {ws_url}...")

    async with websockets.connect(ws_url) as websocket:
        print("Connected to Mission Control. Waiting for tasks...")

        while True:
            try:
                message = await websocket.recv()
                data = json.loads(message)

                if data.get("event") == "NEW_BOOKING":
                    #Extract Data from the Array [x, y, yaw]
                    coords = data.get('coordinates', [])
                    spot_id = data.get('spot_id', 'Unknown')

                    if len(coords) >= 3:
                        x_val = coords[0]
                        y_val = coords[1]
                        yaw_val = coords[2]

                        print(f"\nTask Received for Spot {spot_id}")
                        print(f"   Coordinates: X={x_val}, Y={y_val}, Yaw={yaw_val}")

                        #Create the JSON Payload for your ROS Node
                        nav_payload = {
                            "x": x_val,
                            "y": y_val,
                            "yaw": yaw_val
                        }

                        #Send to Local ROS Node via HTTP POST
                        try:
                            response = requests.post(local_nav_url, json=nav_payload)
                            if response.status_code == 200:
                                print(f"Sent to Robot: Success! ({response.json().get('message')})")
                            else:
                                print(f"Robot Error: {response.text}")
                        except requests.exceptions.ConnectionError:
                            print("Error: Could not connect to Robot (localhost:8080). Is the ROS node running?")
                    else:
                        print(f"Invalid coordinate format received: {coords}")

            except websockets.exceptions.ConnectionClosed:
                print("Connection closed. Reconnecting...")
                break # Break inner loop to trigger restart/reconnect logic

# Run the listener
if __name__ == "__main__":
    while True:
        try:
            asyncio.run(listen_for_tasks())
        except Exception as e:
            print(f"Global Error: {e}. Retrying in 3 seconds...")
            import time
            time.sleep(3)
