using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom using>
using System.Threading;
using System.Threading.Tasks;
// </Custom using>


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
  private void RunScript(bool reset, bool go, List<Point3d> P, List<Vector3d> V, double nR, double coS, double alS, double seS, double seR, ref object Ap, ref object Av)
  {
    
        // <Custom code>
        GH_Point[] ptsOut;
        GH_Vector[] vecOut;

        if (reset || AgSys == null)
        {
            AgSys = new AgentSystem(P, V);
        }

        if (go)
        {
            // update parameters
            AgSys.NeighborhoodRadius = nR;
            AgSys.CohesionStrength = coS;
            AgSys.AlignmentStrength = alS;
            AgSys.SeparationStrength = seS;
            AgSys.SeparationRadius = seR;

            // update system
            AgSys.Update();
            Component.ExpireSolution(true);
        }

        AgSys.GetPtsVecs(out ptsOut, out vecOut);

        Ap = ptsOut;
        Av = vecOut;

        // </Custom code>
    }

    // <Custom additional code> 


    // global variables
    public AgentSystem AgSys;

    // classes

    public class AgentSystem
    {
        public List<Agent> Agents;
        public double NeighborhoodRadius;
        public double CohesionStrength;
        public double AlignmentStrength;
        public double SeparationStrength;
        public double SeparationRadius;
        public double MaxSpeed;
        public double BoundingBoxSize;
        public double ContainmentStrength;

        public AgentSystem(List <Point3d> P, List<Vector3d> V)
        {
            Agents = new List<Agent>();

            for (int i=0; i< P.Count; i++)
            {
                Agent ag = new Agent(P[i], V[i]);
                ag.Flock = this;

                Agents.Add(ag);
            }

            MaxSpeed = 0.3;
            BoundingBoxSize = 30.0;
            ContainmentStrength = 1.0;
        }

        public void Update()
        {
            // . . . . . . . . . . . . . . . . . . . . . . step 1 - calculate desired
            /*
             function(param a, param b, ....) {...}
             Anonymous function
            (param a, param b, ....) => {...}
             */

            // Parallele implementation of a foreach loop
            Parallel.ForEach(Agents, ag =>
            {
                // find neighbours & compute desired velocity for each agent
                ComputeAgentDesiredVelocity(ag);
            });

            //foreach (Agent ag in Agents)
            //{
            //    // find neighbours & compute desired velocity for each agent
            //    ComputeAgentDesiredVelocity(ag);
            //}

            // . . . . . . . . . . . . . . . . . . . . . . step 2 - update agents velocity and position
            Parallel.ForEach(Agents, ag =>
            {
                ag.UpdateVelocityAndPosition();
            });
        }

        public List<Agent> FindNeighbours(Agent ag)
        {
            List<Agent> neighbours = new List<Agent>();

            foreach(Agent neighbour in Agents)
            {
                if (neighbour != ag && neighbour.position.DistanceTo(ag.position) < NeighborhoodRadius)
                    neighbours.Add(neighbour);
            }

            return neighbours;
        }

        public void ComputeAgentDesiredVelocity(Agent ag)
        {
            List<Agent> neighbours = FindNeighbours(ag);
            ag.ComputeDesiredVelocity(neighbours);
        }

        // extract points and vectors for output
        public void GetPtsVecs(out GH_Point[] pts, out GH_Vector[] vecs)
        {
            pts = new GH_Point[Agents.Count];
            vecs = new GH_Vector[Agents.Count];

            for (int i = 0; i < Agents.Count; i++)
            {
                pts[i] = new GH_Point(Agents[i].position);
                vecs[i] = new GH_Vector(Agents[i].velocity);
            }
        }

    }

    public class Agent
    {
        // field
        public Point3d position;
        public Vector3d velocity;
        public Vector3d desiredVelocity;
        public AgentSystem Flock;

        // constructor
        public Agent(Point3d position, Vector3d velocity)
        {
            this.position = position;
            this.velocity = velocity;
            desiredVelocity = this.velocity;
        }

        // methods
        public void ComputeDesiredVelocity(List<Agent> neighbours)
        {

            desiredVelocity = Vector3d.Zero;
            // ------------------------------- CONTAINMENT -------------------------------
            Containment();
            // ------------------------------- FLOCKING -------------------------------
            if (neighbours.Count > 0) 
            {
                // ................................... COHESION BEHAVIOUR .....................
                //
                // find neighbours average
                Point3d average = new Point3d();

                foreach (Agent neighbour in neighbours)
                    average += neighbour.position;

                average /= neighbours.Count;

                // go there
                Vector3d cohesion = average - position;

                desiredVelocity += Flock.CohesionStrength * cohesion;

                // ................................... ALIGNMENT BEHAVIOUR .....................
                //
                Vector3d alignment = Vector3d.Zero;

                foreach (Agent neighbour in neighbours)
                    alignment += neighbour.velocity;

                alignment /= neighbours.Count;

                desiredVelocity += alignment * Flock.AlignmentStrength;

                // ................................... SEPARATION BEHAVIOUR .....................
                //
                Vector3d separation = Vector3d.Zero;

                foreach (Agent neighbour in neighbours)
                {
                    double distanceToNeighbour = position.DistanceTo(neighbour.position);
                    if (distanceToNeighbour < Flock.SeparationRadius)
                    {
                        Vector3d getAway = position - neighbour.position;
                        separation += getAway /= (getAway.Length * distanceToNeighbour);
                    }

                }

                desiredVelocity += separation * Flock.SeparationStrength;
            }

            // ------------------------------- POINT CLOUD BEHAVIOUR -------------------------------

            // ------------------------------- FIELD BEHAVIOUR -------------------------------

            // ------------------------------- CUSTOM MOVEMENT BEHAVIOUR ---------------------
        }

        public void Containment()
        {
            if (position.X < 0.0)
                desiredVelocity += new Vector3d(-position.X, 0.0, 0.0);
            else if (position.X > Flock.BoundingBoxSize)
                desiredVelocity += new Vector3d(Flock.BoundingBoxSize - position.X, 0, 0);

            if (position.Y < 0.0)
                desiredVelocity += new Vector3d(0.0,-position.Y, 0.0);
            else if (position.Y > Flock.BoundingBoxSize)
                desiredVelocity += new Vector3d(0.0, Flock.BoundingBoxSize - position.Y, 0.0);

            if (position.Z < 0.0)
                desiredVelocity += new Vector3d(0.0,0.0,-position.Z);
            else if (position.Z > Flock.BoundingBoxSize)
                desiredVelocity += new Vector3d(0.0,0.0,Flock.BoundingBoxSize - position.Z);

            desiredVelocity *= Flock.ContainmentStrength;
        }

        public void SeekPoint(Point3d target, double intensity)
        {
            Vector3d seek = target - position;
            seek.Unitize();
            seek *= Flock.MaxSpeed;

            desiredVelocity += seek * intensity;
        }

        public void UpdateVelocityAndPosition()
        {
            // steering method
            velocity = 0.97 * velocity + 0.03 * desiredVelocity;

            // limit the velocity to maximum speed (0.5)
            if (velocity.Length > Flock.MaxSpeed)
            {
                velocity.Unitize();
                velocity *= Flock.MaxSpeed;
            }

            position += velocity;
        }

    }

    // utilities functions

    
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
        bool reset = default(bool);
    if (inputs[0] != null)
    {
      reset = (bool)(inputs[0]);
    }

    bool go = default(bool);
    if (inputs[1] != null)
    {
      go = (bool)(inputs[1]);
    }

    Mesh M = default(Mesh);
    if (inputs[2] != null)
    {
      M = (Mesh)(inputs[2]);
    }

    double meI = default(double);
    if (inputs[3] != null)
    {
      meI = (double)(inputs[3]);
    }

    List<Point3d> P = null;
    if (inputs[4] != null)
    {
      P = GH_DirtyCaster.CastToList<Point3d>(inputs[4]);
    }
    List<Vector3d> V = null;
    if (inputs[5] != null)
    {
      V = GH_DirtyCaster.CastToList<Vector3d>(inputs[5]);
    }
    double nR = default(double);
    if (inputs[6] != null)
    {
      nR = (double)(inputs[6]);
    }

    double coS = default(double);
    if (inputs[7] != null)
    {
      coS = (double)(inputs[7]);
    }

    double alS = default(double);
    if (inputs[8] != null)
    {
      alS = (double)(inputs[8]);
    }

    double seS = default(double);
    if (inputs[9] != null)
    {
      seS = (double)(inputs[9]);
    }

    double seR = default(double);
    if (inputs[10] != null)
    {
      seR = (double)(inputs[10]);
    }



    //3. Declare output parameters
      object Ap = null;
  object Av = null;


    //4. Invoke RunScript
    RunScript(reset, go, M, meI, P, V, nR, coS, alS, seS, seR, ref Ap, ref Av);
      
    try
    {
      //5. Assign output parameters to component...
            if (Ap != null)
      {
        if (GH_Format.TreatAsCollection(Ap))
        {
          IEnumerable __enum_Ap = (IEnumerable)(Ap);
          DA.SetDataList(1, __enum_Ap);
        }
        else
        {
          if (Ap is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Ap));
          }
          else
          {
            //assign direct
            DA.SetData(1, Ap);
          }
        }
      }
      else
      {
        DA.SetData(1, null);
      }
      if (Av != null)
      {
        if (GH_Format.TreatAsCollection(Av))
        {
          IEnumerable __enum_Av = (IEnumerable)(Av);
          DA.SetDataList(2, __enum_Av);
        }
        else
        {
          if (Av is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(Av));
          }
          else
          {
            //assign direct
            DA.SetData(2, Av);
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