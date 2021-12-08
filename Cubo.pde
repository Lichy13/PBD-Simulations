PBDSystem crea_cubo(float alto,
    float ancho,
    float profundo,
    float dens,
    int n_alto,
    int n_ancho,
    int n_profundo,
    float stiffness,
    float display_size){
   
  int N = n_alto*n_ancho*n_profundo;
  float masa = dens*alto*ancho*profundo;
  float stiff_distance = 1;
  float stiff_angle_shear = 0;
  float stiff_angle_bending = 0;
  PBDSystem cubo = new PBDSystem(N,masa/N);
  
  float dx = ancho/(n_ancho-1.0);
  float dy = alto/(n_alto-1.0);
  float dz = profundo/(n_profundo-1.0);
  
  int id = 0;
  for(int z = 0; z < n_profundo; z++)
  {
    for (int i = 0; i< n_ancho;i++){
      for(int j = 0; j< n_alto;j++){
      
        Particle p = cubo.particles.get(id);
        p.location.set(dx*i,dy*j,dz*-z);
        p.display_size = display_size;
  
        id++;
      }
    }
  }
  
  /**
   * Creo restricciones de distancia. Aquí sólo se crean restricciones de estructura.
   * Faltarían las de shear y las de bending.
   */
  id = 0;
  for(int z = 0; z < n_profundo; z++)
  {
    for (int i = 0; i< n_ancho;i++){
      for(int j = 0; j< n_alto;j++){
        
        
        /*if(j == n_alto-1)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }*/
        
        //DIAGONALES CARA INFERIOR
        if(i == n_ancho-1-z && j == 0)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho - n_alto);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }
        
        if(i == z && j == 0)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho + n_alto);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }
        
        //DIAGONALES CARA DERECHA
        if(i == n_ancho-1 && j == z)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho + 1);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }
        
        if(i == n_ancho-1 && j == n_alto-1-z)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho - 1);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }
        
        //DIAGONALES CARA SUPERIOR
        if(i == z && j == n_alto-1)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho + n_alto);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }
        
        if(i == n_ancho-1-z && j == n_alto-1)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho - n_alto);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }
        
        //DIAGONALES CARA IZQUIERDA
        if(i == 0 && j == z)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho + 1);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
            if(z > 0)
            {
              Particle par3 = cubo.particles.get(id - n_alto*n_ancho - 1);
                
              Constraint c2 = new FoldConstraint(par,par3,par2,stiff_angle_bending);
              //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
              cubo.add_constraint(c2);
            }
          }
        }
        
        if(i == 0 && j == n_alto-1-z)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho - 1 );
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
            if(z > 0)
            {
              Particle par3 = cubo.particles.get(id - n_alto*n_ancho + 1);
                
              Constraint c2 = new FoldConstraint(par,par3,par2,stiff_angle_bending);
              //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
              cubo.add_constraint(c2);
            }
          }
          
        }
        /*if(z == j && z == i)
        {
          if(z < n_profundo - 1)
          {
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + n_alto*n_ancho + n_alto+ 1);
           // Particle par3 = cubo.particles.get(id+n_alto-1-z);
            //Particle par4 = cubo.particles.get(id+n_alto-1-z + n_alto*n_ancho + n_alto-1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            //Constraint c2 = new DistanceConstraint(par3,par4,dist,stiff_distance);
            cubo.add_constraint(c);
            //cubo.add_constraint(c2);
          } 
        }*/
     
        /*if( z == i && j == n_alto-1-z)
        {
          
          if(z < n_profundo - 1)
          {
            println("i" + i + "j" + j);
            //print("Restricción Diagonal " + id);
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + (n_alto*n_ancho) + n_alto- 1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }*/
        
        /*if( j == n_ancho-1-z && i == n_ancho-1-z)
        {
          
          if(z < n_profundo - 1)
          {
            println("i" + i + "j" + j);
            //print("Restricción Diagonal " + id);
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + (n_alto*n_ancho) - n_alto- 1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }*/
        
        /*if( z == j && i == n_alto-1-z)
        {
          
          if(z < n_profundo - 1)
          {
            println("i" + i + "j" + j);
            //print("Restricción Diagonal " + id);
            Particle par = cubo.particles.get(id);
            Particle par2 = cubo.particles.get(id + (n_alto*n_ancho) - n_alto+ 1);
            float dist = PVector.sub(par2.location, par.location).mag();
            Constraint c = new DistanceConstraint(par,par2,dist,stiff_distance);
            cubo.add_constraint(c);
          }
        }*/
        //println("id: "+id+" (i,j) = ("+i+","+j+")");
        Particle p = cubo.particles.get(id);
        if(i>0){
          int idx = id - n_alto;
          Particle px = cubo.particles.get(idx);
          Constraint c = new DistanceConstraint(p,px,dx,stiff_distance);
          cubo.add_constraint(c);
          if(j>0)
          {
            int idyt = id-1;
            int idxt = id -n_alto;
            int idxyt = id - n_alto -1 ;
            Particle pyt = cubo.particles.get(idyt);
            Particle pxt = cubo.particles.get(idxt);
            Particle pxyt = cubo.particles.get(idxyt);
            Constraint f1 = new FoldConstraint(p,pxt,pyt,stiff_angle_shear);
            Constraint f2 = new FoldConstraint(pxt,pxyt,p,stiff_angle_shear);
            Constraint f3 = new FoldConstraint(pxyt,pyt,pxt,stiff_angle_shear);
            Constraint f4 = new FoldConstraint(pyt,p,pxyt,stiff_angle_shear);
            cubo.add_constraint(f1);
            cubo.add_constraint(f2);
            cubo.add_constraint(f3);
            cubo.add_constraint(f4);
            //println("Restricción Triangulo");
            //println("T1: "+ id+"->"+  "->"+ idxt + "->" + idyt );
            //println("T2: "+ idxt+"->"+  "->"+ idxyt + "->" + id );
            //println("T3: "+ idxyt+"->"+  "->"+ idyt + "->" + idxt );
            //println("T4: "+ idyt+"->"+  "->"+ id + "->" + idxyt );
          }
          
          if(i < n_ancho - 1)
          {
            int idxiz = id - n_alto;
            int idxder = id + n_alto;
            Particle iz = cubo.particles.get(idxiz);
            Particle de = cubo.particles.get(idxder);
            Constraint shear = new FoldConstraint(p,iz,de,stiff_angle_bending);
            cubo.add_constraint(shear);
          }
          
          /*if(j == n_alto-1 && z > 0)
          {
            
              int idyt = id - n_alto*n_ancho;
              int idxt = id - n_alto;
              int idxyt = id - n_alto*n_ancho - n_alto ;
              Particle pyt = cubo.particles.get(idyt);
              Particle pxt = cubo.particles.get(idxt);
              Particle pxyt = cubo.particles.get(idxyt);
              Constraint f1 = new FoldConstraint(p,pxt,pyt,stiff_angle_shear);
              Constraint f2 = new FoldConstraint(pxt,pxyt,p,stiff_angle_shear);
              Constraint f3 = new FoldConstraint(pxyt,pyt,pxt,stiff_angle_shear);
              Constraint f4 = new FoldConstraint(pyt,p,pxyt,stiff_angle_shear);
              cubo.add_constraint(f1);
              cubo.add_constraint(f2);
              cubo.add_constraint(f3);
              cubo.add_constraint(f4);
                
          }*/
          
        }
        
        
  
        if(j>0){
          int idy = id - 1;
          Particle py = cubo.particles.get(idy);
          Constraint c = new DistanceConstraint(p,py,dy,stiff_distance);
          cubo.add_constraint(c);
          //println("Restricción creada: "+ id+"->"+idy);
          
          if (j < n_alto - 1)
          {
            int idarriba = id - 1;
            int idabajo = id + 1;
            Particle arriba = cubo.particles.get(idarriba);
            Particle abajo = cubo.particles.get(idabajo);
            Constraint shear = new FoldConstraint(p,arriba,abajo,stiff_angle_bending);
            cubo.add_constraint(shear);
          }
          
          /*if(i == 0 && z < n_profundo - 1)
          {
            int idyt = id - 1;
            int idxt = id + n_alto*n_ancho;
            int idxyt = id + n_alto*n_ancho - 1 ;
            Particle pyt = cubo.particles.get(idyt);
            Particle pxt = cubo.particles.get(idxt);
            Particle pxyt = cubo.particles.get(idxyt);
            Constraint f1 = new FoldConstraint(p,pxt,pyt,stiff_angle_shear);
            Constraint f2 = new FoldConstraint(pxt,pxyt,p,stiff_angle_shear);
            Constraint f3 = new FoldConstraint(pxyt,pyt,pxt,stiff_angle_shear);
            Constraint f4 = new FoldConstraint(pyt,p,pxyt,stiff_angle_shear);
            cubo.add_constraint(f1);
            cubo.add_constraint(f2);
            cubo.add_constraint(f3);
            cubo.add_constraint(f4);
              
          }
          
          if(i == n_ancho-1 && z < n_profundo - 1)
          {
            int idyt = id - 1;
            int idxt = id + n_alto*n_ancho;
            int idxyt = id + n_alto*n_ancho - 1 ;
            Particle pyt = cubo.particles.get(idyt);
            Particle pxt = cubo.particles.get(idxt);
            Particle pxyt = cubo.particles.get(idxyt);
            Constraint f1 = new FoldConstraint(p,pxt,pyt,stiff_angle_shear);
            Constraint f2 = new FoldConstraint(pxt,pxyt,p,stiff_angle_shear);
            Constraint f3 = new FoldConstraint(pxyt,pyt,pxt,stiff_angle_shear);
            Constraint f4 = new FoldConstraint(pyt,p,pxyt,stiff_angle_shear);
            cubo.add_constraint(f1);
            cubo.add_constraint(f2);
            cubo.add_constraint(f3);
            cubo.add_constraint(f4);
              
          }*/
        }
  
        id++;
        }
    }
  }
  
  // Fijamos dos esquinas
  /*for(int z = 0; z < n_profundo; z++)
  {
    id = n_alto-1 + z*(n_alto*n_ancho);
    cubo.particles.get(id).set_bloqueada(true); 
    
    id = n_alto*n_ancho - 1 + z*(n_alto*n_ancho);
    cubo.particles.get(id).set_bloqueada(true); 
      
  }*/
  
  //print("Tela creada con " + cubo.particles.size() + " partículas y " + cubo.constraints.size() + " restricciones."); 
  
  return cubo;
}
