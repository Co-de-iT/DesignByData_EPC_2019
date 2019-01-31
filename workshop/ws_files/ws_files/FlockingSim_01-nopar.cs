using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;



/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { __out.Add(text); }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private IGH_Component Component; 
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments, 
  /// Output parameters as ref arguments. You don't have to assign output parameters, 
  /// they will have a default value.
  /// </summary>
  private void RunScript(bool Reset, bool Play, bool is3D, int Count, double TimeStep, double NeighborRad, double Alignment, double Cohesion, double Separation, double SepDist, List<Circle> Repellers, bool UseParallel, bool UseRTree, ref object Positions, ref object Velocities)
  {
        // <Custom code>

        // adapted from Long Nguyen ICD C# ws 2017
        // adaptation by Alessio Erioli - (c) Co-de-iT 2019
        // part 01 - flocking without parallelization

        List<GH_Point> positions = new List<GH_Point>();
        List<GH_Vector> velocities = new List<GH_Vector>();

        if (Reset || flockSystem == null)
            flockSystem = new FlockSystem(Count, is3D);

        else
        {
            flockSystem.TimeStep = TimeStep;
            flockSystem.NeighborhoodRadius = NeighborRad;
            flockSystem.AlignmentStrength = Alignment;
            flockSystem.CohesionStrength = Cohesion;
            flockSystem.SeparationStrength = Separation;
            flockSystem.SeparationDistance = SepDist;
            flockSystem.Repellers = Repellers;
            flockSystem.UseParallel = UseParallel;
            flockSystem.UseRTree = UseRTree;

            flockSystem.Update();

            if (Play) Component.ExpireSolution(true);

            foreach (FlockAgent agent in flockSystem.Agents)
            {
                positions.Add(new GH_Point(agent.Position));
                velocities.Add(new GH_Vector(agent.Velocity));
            }


            Positions = positions;
            Velocities = velocities;

        }

        // </Custom code>
    }

    // <Custom additional code> 

    private FlockSystem flockSystem;


    // ---------- Flock class ----------

    public class FlockSystem
    {
        public List<FlockAgent> Agents;

        public double TimeStep;
        public double NeighborhoodRadius;
        public double FieldOfView;
        public double AlignmentStrength;
        public double CohesionStrength;
        public double SeparationStrength;
        public double SeparationDistance;
        public double MinSpeed;
        public double MaxSpeed;
        public List<Circle> Repellers;
        public bool UseParallel;
        public bool UseRTree;

        public FlockSystem(int agentCount, bool is3D)
        {
            Agents = new List<FlockAgent>();

            if (is3D) 
            {
               for (int i = 0; i < agentCount; i++)
                {
                    FlockAgent agent = new FlockAgent(
                        Util.GetRandomPoint(0.0, 30.0, 0.0, 30.0, 0.0, 30.0),
                        Util.GetRandomUnitVector() * 4.0);
                    agent.FlockSystem = this;

                    Agents.Add(agent);
                } 
            }
            else
                for(int i=0; i< agentCount; i++)
                {
                    FlockAgent agent = new FlockAgent(
                        Util.GetRandomPoint(0.0, 30.0, 0.0, 30.0, 0.0, 0.0),
                        Util.GetRandomUnitVectorXY() * 4.0);
                    agent.FlockSystem = this;

                    Agents.Add(agent);
                }
        }

        private List<FlockAgent> FindNeighBours(FlockAgent agent)
        {
            List<FlockAgent> neighbours = new List<FlockAgent>();

            foreach (FlockAgent neighbour in Agents)
            {
                if (neighbour != agent && neighbour.Position.DistanceTo(agent.Position) < NeighborhoodRadius)
                    neighbours.Add(neighbour);
            }

            return neighbours;
        }

        public void Update()
        {
            foreach(FlockAgent agent in Agents)
            {
                List<FlockAgent> neighbours = FindNeighBours(agent);
                agent.ComputeDesiredVelocity(neighbours);
            }

            foreach (FlockAgent agent in Agents)
                agent.UpdateVelocityAndPosition();
        }

        
    }

    // ---------- Agent class ----------

    public class FlockAgent
    {
        public Point3d Position;
        public Vector3d Velocity;
        private Vector3d desiredVelocity;

        public FlockSystem FlockSystem;

        public FlockAgent(Point3d position, Vector3d velocity)
        {
            Position = position;
            Velocity = velocity;
        }

        public void UpdateVelocityAndPosition()
        {
            Velocity = 0.97 * Velocity + 0.03 * desiredVelocity;

            if (Velocity.Length > 8.0) Velocity *= 8.0 / Velocity.Length;
            else if (Velocity.Length < 4.0) Velocity *= 4.0 / Velocity.Length;

            Position += Velocity * FlockSystem.TimeStep;

        }

        public void ComputeDesiredVelocity(List<FlockAgent> neighbours)
        {
            desiredVelocity = new Vector3d(0.0, 0.0, 0.0);

            // Containment behavior --------------------

            double boundingBoxSize = 30.0;

            if (Position.X < 0.0)
                desiredVelocity += new Vector3d(-Position.X, 0.0, 0.0);
            else if (Position.X > boundingBoxSize)
                desiredVelocity += new Vector3d(boundingBoxSize - Position.X, 0.0, 0.0);
            if (Position.Y < 0.0)
                desiredVelocity += new Vector3d(0.0,-Position.Y, 0.0);
            else if (Position.Y > boundingBoxSize)
                desiredVelocity += new Vector3d(0.0, boundingBoxSize - Position.Y, 0.0);
            if (Position.Z < 0.0)
                desiredVelocity += new Vector3d( 0.0, 0.0, -Position.Z);
            else if (Position.Z > boundingBoxSize)
                desiredVelocity += new Vector3d(0.0, 0.0, boundingBoxSize - Position.Z);

            if (neighbours.Count == 0)
                desiredVelocity += Velocity; // maintain current velocity
            else
            {
                // Alignment behavior --------------------

                Vector3d alignment = Vector3d.Zero;

                foreach (FlockAgent neighbour in neighbours)
                    alignment += neighbour.Velocity;

                alignment /= neighbours.Count;
                desiredVelocity += FlockSystem.AlignmentStrength * alignment;

                // Cohesion behavior --------------------

                Point3d centre = Point3d.Origin; // a zero point to count average neighbours position

                foreach (FlockAgent neighbour in neighbours)
                    centre += neighbour.Position;

                centre /= neighbours.Count;

                Vector3d cohesion = centre - Position;
                desiredVelocity += FlockSystem.CohesionStrength * cohesion;

                // Separation behavior --------------------

                Vector3d separation = Vector3d.Zero;

                foreach (FlockAgent neighbour in neighbours)
                {
                    double distanceToNeighbour = Position.DistanceTo(neighbour.Position);
                    if(distanceToNeighbour < FlockSystem.SeparationDistance)
                    {
                        Vector3d getAway = Position - neighbour.Position;
                        separation += getAway /= (getAway.Length * distanceToNeighbour);
                    }

                }

                desiredVelocity += FlockSystem.SeparationStrength * separation;

            }

            // Avoiding obstacles (repellers) --------------------

            foreach (Circle repeller in FlockSystem.Repellers)
            {
                double distanceToRepeller = Position.DistanceTo(repeller.Center);
                Vector3d repulsion = Position - repeller.Center;

                repulsion /= (repulsion.Length * distanceToRepeller);
                repulsion *= 30.0 * repeller.Radius;
                desiredVelocity += repulsion;
            }
        }
    }

    // ---------- Utilities class ----------

    public static class Util
    {
        static Random random = new Random();

        public static Point3d GetRandomPoint(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
        {
            double x = minX + (maxX - minX) * random.NextDouble();
            double y = minY + (maxY - minY) * random.NextDouble();
            double z = minZ + (maxZ - minZ) * random.NextDouble();

            return new Point3d(x, y, z);
        }

        public static Vector3d GetRandomUnitVector()
        {
            double phi = 2.0 * Math.PI * random.NextDouble();
            double theta = Math.Acos(2.0 * random.NextDouble() - 1.0);

            double x = Math.Sin(theta) * Math.Cos(phi);
            double y = Math.Sin(theta) * Math.Sin(phi);
            double z = Math.Cos(theta);

            return new Vector3d(x, y, z);
        }

        public static Vector3d GetRandomUnitVectorXY()
        {
            double angle = 2.0 * Math.PI * random.NextDouble();

            double x = Math.Cos(angle);
            double y = Math.Sin(angle);

            return new Vector3d(x, y, 0.0);
        }
    }
    // </Custom additional code> 

    private List<string> __err = new List<string>(); //Do not modify this list directly.
  private List<string> __out = new List<string>(); //Do not modify this list directly.
  private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
  private IGH_ActiveObject owner;                  //Legacy field.
  private int runCount;                            //Legacy field.
  
  public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
  {
    //Prepare for a new run...
    //1. Reset lists
    this.__out.Clear();
    this.__err.Clear();

    this.Component = owner;
    this.Iteration = iteration;
    this.GrasshopperDocument = owner.OnPingDocument();
    this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

    this.owner = this.Component;
    this.runCount = this.Iteration;
    this. doc = this.RhinoDocument;

    //2. Assign input parameters
        bool Reset = default(bool);
    if (inputs[0] != null)
    {
      Reset = (bool)(inputs[0]);
    }

    bool Play = default(bool);
    if (inputs[1] != null)
    {
      Play = (bool)(inputs[1]);
    }

    bool is3D = default(bool);
    if (inputs[2] != null)
    {
      is3D = (bool)(inputs[2]);
    }

    int Count = default(int);
    if (inputs[3] != null)
    {
      Count = (int)(inputs[3]);
    }

    double Timestep = default(double);
    if (inputs[4] != null)
    {
      Timestep = (double)(inputs[4]);
    }

    double NeighborRad = default(double);
    if (inputs[5] != null)
    {
      NeighborRad = (double)(inputs[5]);
    }

    double Alignment = default(double);
    if (inputs[6] != null)
    {
      Alignment = (double)(inputs[6]);
    }

    double Cohesion = default(double);
    if (inputs[7] != null)
    {
      Cohesion = (double)(inputs[7]);
    }

    double Separation = default(double);
    if (inputs[8] != null)
    {
      Separation = (double)(inputs[8]);
    }

    double SepDist = default(double);
    if (inputs[9] != null)
    {
      SepDist = (double)(inputs[9]);
    }

    List<Circle> Repellers = null;
    if (inputs[10] != null)
    {
      Repellers = GH_DirtyCaster.CastToList<Circle>(inputs[10]);
    }
    bool UseParallel = default(bool);
    if (inputs[11] != null)
    {
      UseParallel = (bool)(inputs[11]);
    }

    bool UseRTree = default(bool);
    if (inputs[12] != null)
    {
      UseRTree = (bool)(inputs[12]);
    }



    //3. Declare output parameters
      object Positions = null;
  object Velocities = null;


    //4. Invoke RunScript
    RunScript(Reset, Play, is3D, Count, Timestep, NeighborRad, Alignment, Cohesion, Separation, SepDist, Repellers, UseParallel, UseRTree, ref Positions, ref Velocities);
      
    try
    {
      //5. Assign output parameters to component...
            if (Positions != null)
      {
        if (GH_Format.TreatAsCollection(Positions))
        {
          IEnumerable __enum_Positions = (IEnumerable)(Positions);
          DA.SetDataList(1, __enum_Positions);
        }
        else
        {
          if (Positions is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Positions));
          }
          else
          {
            //assign direct
            DA.SetData(1, Positions);
          }
        }
      }
      else
      {
        DA.SetData(1, null);
      }
      if (Velocities != null)
      {
        if (GH_Format.TreatAsCollection(Velocities))
        {
          IEnumerable __enum_Velocities = (IEnumerable)(Velocities);
          DA.SetDataList(2, __enum_Velocities);
        }
        else
        {
          if (Velocities is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(Velocities));
          }
          else
          {
            //assign direct
            DA.SetData(2, Velocities);
          }
        }
      }
      else
      {
        DA.SetData(2, null);
      }

    }
    catch (Exception ex)
    {
      this.__err.Add(string.Format("Script exception: {0}", ex.Message));
    }
    finally
    {
      //Add errors and messages... 
      if (owner.Params.Output.Count > 0)
      {
        if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
        {
          List<string> __errors_plus_messages = new List<string>();
          if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
          if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
          if (__errors_plus_messages.Count > 0) 
            DA.SetDataList(0, __errors_plus_messages);
        }
      }
    }
  }
}