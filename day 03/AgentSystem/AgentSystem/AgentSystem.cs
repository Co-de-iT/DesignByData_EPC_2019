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
    private void RunScript(bool reset, bool go, List<Point3d> P, List<Vector3d> V, double nR, double cS, ref object Ap, ref object Av)
    {
        // <Custom code>

        GH_Point[] ptsOut;
        GH_Vector[] vecOut;

        if (reset || Flock == null)
        {
            Flock = new FlockSystem(P, V);
            //ptsOut = new GH_Point[Flock.Agents.Count];
            //vecOut = new GH_Vector[Flock.Agents.Count];
        }

        if (go)
        {
            // update parameters
            Flock.NeighborhoodRadius = nR;
            Flock.CohesionStrength = cS;

            // update system
            Flock.Update();
            Component.ExpireSolution(true);
        }

        Flock.GetPtsVecs(out ptsOut, out vecOut);

        Ap = ptsOut;
        Av = vecOut;

        // </Custom code>
    }

    // <Custom additional code> 

    // global variables
    public FlockSystem Flock;


    // classes

    public class FlockSystem
    {
        public List<Agent> Agents;
        public double NeighborhoodRadius;
        public double CohesionStrength;

        public FlockSystem(List <Point3d> P, List<Vector3d> V)
        {
            Agents = new List<Agent>();

            for (int i=0; i< P.Count; i++)
            {
                Agent ag = new Agent(P[i], V[i]);
                ag.Flock = this;

                Agents.Add(ag);
            }
        }

        public void Update()
        {
            // . . . . . . . . . . . . . . . . . . . . . . step 1 - calculate desired

            foreach (Agent ag in Agents)
            {
                // find neighbours & compute desired velocity for each agent
                ComputeAgentDesiredVelocity(ag);
            }

            // . . . . . . . . . . . . . . . . . . . . . . step 2 - update agents velocity and position
            foreach (Agent ag in Agents) ag.UpdateVelocityAndPosition();
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
        public FlockSystem Flock;

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


            if (neighbours.Count == 0)
                desiredVelocity = velocity;
            else
            {
                // find neighbours average
                Point3d average = new Point3d();

                foreach (Agent neighbour in neighbours)
                    average += neighbour.position;

                average /= neighbours.Count;

                // go there
                Vector3d cohesion = average - position;

                desiredVelocity += Flock.CohesionStrength * cohesion;

            }
        }

        public void UpdateVelocityAndPosition()
        {
            // steering method
            velocity = 0.97 * velocity + 0.03 * desiredVelocity;

            // limit the velocity to maximum speed (4.0)
            if (velocity.Length > 4.0)
            {
                velocity.Unitize();
                velocity *= 4.0;
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
        this.doc = this.RhinoDocument;

        //2. Assign input parameters
        object x = default(object);
        if (inputs[0] != null)
        {
            x = (object)(inputs[0]);
        }

        object y = default(object);
        if (inputs[1] != null)
        {
            y = (object)(inputs[1]);
        }



        //3. Declare output parameters
        object A = null;


        //4. Invoke RunScript
        RunScript(x, y, ref A);

        try
        {
            //5. Assign output parameters to component...
            if (A != null)
            {
                if (GH_Format.TreatAsCollection(A))
                {
                    IEnumerable __enum_A = (IEnumerable)(A);
                    DA.SetDataList(1, __enum_A);
                }
                else
                {
                    if (A is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(A));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, A);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
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