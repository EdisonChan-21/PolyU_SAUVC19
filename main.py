import asyncio
import testDetection as detect
import controllor
import cv2
import numpy as np
import gateDetector as gate
import time
import handler
import imageManager as im
# import videoStream as vs

async def runFlow(control):
    # im.taskTest(control)
    await im.task1(control)

async def main():
    mainControl = controllor.control()
    # video = vs.Video()
    await asyncio.gather(
        mainControl.test(),
        runFlow(mainControl)
    )

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
