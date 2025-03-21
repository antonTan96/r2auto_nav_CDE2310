class MapNode():
    def __init__(self,x:float,y:float):
        self.x = float(x)
        self.y = float(y)
        self.parent = None
        self.neighbors = []

    def __eq__(self, other):
        if isinstance(other, MapNode):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))
    
    def __str__(self):
        return "X: "+str(self.x)+" Y: "+str(self.y)

    def generate_neighbors(self,max_x,max_y):
        if len(self.neighbors)==0:
            for i in range(4):
                if i==0:
                    if self.x>0:
                        self.neighbors.append(MapNode(self.x-1,self.y))
                if i==1:
                    if self.x<max_x-1:
                        self.neighbors.append(MapNode(self.x+1,self.y))
                if i==2:
                    if self.y>0:
                        self.neighbors.append(MapNode(self.x,self.y-1))
                if i==3:
                    if self.y<max_y-1:
                        self.neighbors.append(MapNode(self.x,self.y+1))
            return self.neighbors
        else:
            return self.neighbors



            

