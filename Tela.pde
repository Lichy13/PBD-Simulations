

PBDSystem crea_tela(float alto,
    float ancho,
    float dens,
    int n_alto,
    int n_ancho,
    float stiffness,
    float display_size){
   
  int N = n_alto*n_ancho;
  float masa = dens*alto*ancho;
  float stiff_distance = 0.2;
  float stiff_angle_shear = 0;
  float stiff_angle_bending = 0;
  PBDSystem tela = new PBDSystem(N,masa/N);
  
  float dx = ancho/(n_ancho-1.0);
  float dy = alto/(n_alto-1.0);
  
  int id = 0;
  if(tela.ActiveSphere)
  {
    for (int i = 0; i< n_ancho;i++){
      for(int j = 0; j< n_alto;j++){
        Particle p = tela.particles.get(id);
        p.location.set(dx*i,0.5,-dy*j);
        p.display_size = display_size;
  
        id++;
      }
    }
  }
  else
  {
    for (int i = 0; i< n_ancho;i++){
      for(int j = 0; j< n_alto;j++){
        Particle p = tela.particles.get(id);
        p.location.set(dx*i,dy*j,0);
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
  for (int i = 0; i< n_ancho;i++){
    for(int j = 0; j< n_alto;j++){
      println("id: "+id+" (i,j) = ("+i+","+j+")");
      Particle p = tela.particles.get(id);
      if(i>0){
        int idx = id - n_alto;
        Particle px = tela.particles.get(idx);
        Constraint c = new DistanceConstraint(p,px,dx,stiff_distance);
        tela.add_constraint(c);
        if(j>0)
        {
          int idyt = id-1;
          int idxt = id -n_alto;
          int idxyt = id - n_alto -1 ;
          Particle pyt = tela.particles.get(idyt);
          Particle pxt = tela.particles.get(idxt);
          Particle pxyt = tela.particles.get(idxyt);
          Constraint f1 = new FoldConstraint(p,pxt,pyt,stiff_angle_shear);
          Constraint f2 = new FoldConstraint(pxt,pxyt,p,stiff_angle_shear);
          Constraint f3 = new FoldConstraint(pxyt,pyt,pxt,stiff_angle_shear);
          Constraint f4 = new FoldConstraint(pyt,p,pxyt,stiff_angle_shear);
          tela.add_constraint(f1);
          tela.add_constraint(f2);
          tela.add_constraint(f3);
          tela.add_constraint(f4);
          println("Restricción Triangulo");
          println("T1: "+ id+"->"+  "->"+ idxt + "->" + idyt );
          println("T2: "+ idxt+"->"+  "->"+ idxyt + "->" + id );
          println("T3: "+ idxyt+"->"+  "->"+ idyt + "->" + idxt );
          println("T4: "+ idyt+"->"+  "->"+ id + "->" + idxyt );
        }
        
        if(i < n_ancho - 1)
        {
          int idxiz = id - n_alto;
          int idxder = id + n_alto;
          Particle iz = tela.particles.get(idxiz);
          Particle de = tela.particles.get(idxder);
          Constraint shear = new FoldConstraint(p,iz,de,stiff_angle_bending);
          tela.add_constraint(shear);
        }
        
      }

      if(j>0){
        int idy = id - 1;
        Particle py = tela.particles.get(idy);
        Constraint c = new DistanceConstraint(p,py,dy,stiff_distance);
        tela.add_constraint(c);
        println("Restricción creada: "+ id+"->"+idy);
        
        if (j < n_alto - 1)
        {
          int idarriba = id - 1;
          int idabajo = id + 1;
          Particle arriba = tela.particles.get(idarriba);
          Particle abajo = tela.particles.get(idabajo);
          Constraint shear = new FoldConstraint(p,arriba,abajo,stiff_angle_bending);
          tela.add_constraint(shear);
        }
      }

      id++;
    }
  }
  
  // Fijamos dos esquinas
  if(!tela.ActiveSphere)
  {
     id = n_alto-1;
    tela.particles.get(id).set_bloqueada(true); 
    
    id = N-1;
    tela.particles.get(id).set_bloqueada(true); 
  }
  
  print("Tela creada con " + tela.particles.size() + " partículas y " + tela.constraints.size() + " restricciones."); 
  
  return tela;
}
