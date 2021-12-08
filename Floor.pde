class Floor{
PVector x1;
PVector x2;
PVector x3; 
PVector x4;
PVector normal;
  
  Floor()
  {
    x1 = new PVector(10,-0.2,10);
    x2 = new PVector(10,-0.2,-10);
    x3 = new PVector(-10,-0.2,-10); 
    x4 = new PVector(-10,-0.2,10);
    normal = new PVector(0,1,0);    
  }
  void drawFloor()
  {
    beginShape();
    vertex(x1.x*scale_px,-x1.y*scale_px,x1.z*scale_px);
    vertex(x2.x*scale_px,-x2.y*scale_px,x2.z*scale_px);
    vertex(x3.x*scale_px,-x3.y*scale_px,x3.z*scale_px);
    vertex(x4.x*scale_px,-x4.y*scale_px,x4.z*scale_px);
    endShape(CLOSE);
  }

  boolean IsColliding(Particle p)
  {
   boolean isColliding = false;
   float dcol = PVector.sub(p.location,x1).dot(normal);
   float d = dcol - sphere_size_tela;
   //println("loc "+ p.location + " x1 " +  x1 + "dcol " + d);
   if(dcol - sphere_size_tela < 0)
   {
     isColliding = true;
   }
   return isColliding;
  }
}
