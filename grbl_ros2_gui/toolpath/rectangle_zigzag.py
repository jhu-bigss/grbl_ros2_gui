

class RectangleZigzagPath:

    def __init__(self, tool_diameter=1.0, stepover_percentage=1.0, scan_speed=1000.0):
        self.toolDiam = tool_diameter
        self.stepoverPercentage = stepover_percentage
        self.scanSpeed = scan_speed

    def make(self, X_start=0.0, Y_start=0.0, width=10., height=10., fill_type='Raster'):

        # GCode Blocks
        blocks = []

        # Check parameters
        if fill_type == "":
            print("Flatten abort: Pocket Type is undefined")
            return

        if width <= 0 or height <= 0 :
            print("Flatten abort: Flatten Area dimensions must be > 0")
            return
        
        # # Add Region disabled to show worked area
        # xR,yR = self.RectPath(X_start, Y_start, width, height)
        # for x,y in zip(xR,yR):
        #     print("x: %.1f, y: %.1f"%(x, y))

        # Calc tool diameter with Maximum Step Over allowed
        StepOverInUnitMax = self.toolDiam * self.stepoverPercentage

        # Offset Border to compensate tool radius
        toolRadius = self.toolDiam / 2.

        BorderX_start = X_start + toolRadius
        BorderY_start = Y_start + toolRadius
        BorderWidth = width - self.toolDiam
        BorderHeight = height - self.toolDiam
        BorderXEnd = X_start + width - toolRadius
        BorderYEnd = Y_start + height - toolRadius

        # Calc space to work with/without border cut
        WToWork = width - self.toolDiam
        HToWork = height - self.toolDiam

        if(WToWork < toolRadius or HToWork < toolRadius):
            print("[Error] Abort: rectangular area is too small to fill.")
            return

        # Prepare points for pocketing
        xP=[]
        yP=[]

        #---------------------------------------------------------------------
        # Raster approach
        if fill_type == "Raster":
            # Calc number of pass
            VerticalCount = (int)(HToWork / StepOverInUnitMax)
            # Calc step minor of Max step
            StepOverInUnit = HToWork / (VerticalCount +1)
            flip = False
            ActualY = BorderY_start
            # Zig zag
            if StepOverInUnit==0 : StepOverInUnit=0.001  #avoid infinite while loop
            while (True):
                # Zig
                xP.append(self.ZigZag(flip,BorderX_start,BorderXEnd))
                yP.append(ActualY)
                flip = not flip
                # Zag
                xP.append(self.ZigZag(flip,BorderX_start,BorderXEnd))
                yP.append(ActualY)
                if(ActualY >= BorderYEnd - StepOverInUnitMax + StepOverInUnit):
                    break
                # Up
                ActualY += StepOverInUnit
                xP.append(self.ZigZag(flip,BorderX_start,BorderXEnd))
                yP.append(ActualY)

        #---------------------------------------------------------------------
        # Offset approach
        if fill_type == "Offset":
            # Calc number of pass
            VerticalCount = (int)(HToWork / StepOverInUnitMax)
            HorrizontalCount = (int)(WToWork / StepOverInUnitMax)
            # Make them odd
            if VerticalCount%2 == 0 : VerticalCount += 1
            if HorrizontalCount%2 == 0 : HorrizontalCount += 1
            # Calc step minor of Max step
            StepOverInUnitH = HToWork / (VerticalCount)
            StepOverInUnitW = WToWork / (HorrizontalCount)

            # Start from border to center
            xS = BorderX_start
            yS = BorderY_start
            wS = WToWork
            hS = HToWork
            xC = 0
            yC = 0
            while (xC<=HorrizontalCount/2 and yC<=VerticalCount/2):
                # Pocket offset points
                xO,yO = self.RectPath(xS, yS, wS, hS)
                xP = xP + xO
                yP = yP + yO
                xS+=StepOverInUnitH
                yS+=StepOverInUnitW
                hS-=2.0*StepOverInUnitH
                wS-=2.0*StepOverInUnitW
                xC += 1
                yC += 1

            # Reverse point to start from inside (less stress on the tool)
            xP = xP[::-1]
            yP = yP[::-1]

        # Generate Gcode blocks for zigzag, move safe to first point
        gcode = "G0"
        gcode += "X{:0.3f}Y{:0.3f}".format(xP[0],yP[0])
        blocks.append(gcode)

        # Pocketing
        lastxy = None
        for x,y in zip(xP,yP):
            gcode = "G1"
            if lastxy != (x,y) or None:
                gcode += "X{:0.3f}Y{:0.3f}".format(x,y)
                gcode += "F{:0.2f}".format(self.scanSpeed)
                blocks.append(gcode)
            lastxy = (x,y)

        return blocks

    #----------------------------------------------------------------------
    def RectPath(self,x,y,w,h):
        xR = []
        yR = []
        xR.append(x)
        yR.append(y)
        xR.append(x + w)
        yR.append(y)
        xR.append(x + w)
        yR.append(y + h)
        xR.append(x)
        yR.append(y + h)
        xR.append(x)
        yR.append(y)
        return (xR,yR)

    #----------------------------------------------------------------------
    def ZigZag(self,flip,zig,zag):
        if flip:
            return zig
        else:
            return zag 
