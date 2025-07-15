import asyncio
import websocket
import rel

from ExperimentOrchestrator.Architecture.Distributed.RRWebSocketClient import RRWebSocketClient
from Plugins.Systems.Docker.DockerRunner import DockerRunner

class RL4GreenROS_RR_Nav2(RRWebSocketClient):

    docker_runner: DockerRunner = None

    async def clean_docker(self):
        print('Clean Docker!')
        containers = []
        containers.append("nav2")
        self.docker_runner.set_containers(containers)
        await self.docker_runner.remove_containers()

    async def start_remote_nav2_container(self):
        print('Starting Nav2 container!')
        await self.docker_runner.start_container("nav2")
        
    async def start_nav2_measurement(self):
        print('Start Nav 2 measurements!')
        # Your code here

    async def stop_nav2_measurement(self):
        print('Stop Nav 2 measurements!')
        # Your code here

if __name__ == "__main__": 
    websocket.enableTrace(False)
    ws_client = RL4GreenROS_RR_Nav2("ws://server.robot-runner:8765")
    ws_client.docker_runner = DockerRunner()
    ws_client.run_forever()
    ws_client.register("nav2")
    rel.signal(2, rel.abort) 
    rel.dispatch()
