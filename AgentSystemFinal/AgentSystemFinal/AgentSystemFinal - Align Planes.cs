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
public class Script_Instance2 : GH_ScriptInstance
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
  private void RunScript(bool reset, bool go, List<Plane> P, double aligRadius, double aligStrength, ref object POut)
  {
        // <Custom code>

        // return if no planes are supplied
        if (P == null) return;

        // initialize AgentPlaneSystem on reset or at first component execution
        if (reset || AgentPlaneSystem == null )
        {
            AgentPlaneSystem = new AgentSystem(P);
        }

        // Update on go
        if (go)
        {
            AlignRadius = aligRadius;
            AlignStrength = aligStrength;

            AgentPlaneSystem.Update();

            Component.ExpireSolution(true);
        }

        // output geometry
        POut = AgentPlaneSystem.GetPlanes();

        // </Custom code>
    }

    // <Custom additional code> 

    public static double AlignRadius;
    public static double AlignStrength;
    public AgentSystem AgentPlaneSystem;

    /// <summary>
    /// This class manages a List of AgentPlane
    /// </summary>
    public class AgentSystem
    {
        public List<AgentPlane> AgentPlanes;
        public RTree PlanesRTree;

        public AgentSystem(List<Plane> Planes)
        {
            AgentPlanes = new List<AgentPlane>();
            PlanesRTree = new RTree();

            for (int i=0; i< Planes.Count; i++)
            {
                AgentPlanes.Add(new AgentPlane(Planes[i]));
                PlanesRTree.Insert(Planes[i].Origin, i);
            }
        }

        /// <summary>
        /// Upate locates neighbours via RTree and calls eacch AgentPlane AlignWithNeighbours method
        /// </summary>
        public void Update()
        {
            foreach (AgentPlane ap in AgentPlanes)
            {
                List<AgentPlane> neighbours = new List<AgentPlane>();

                // Eventhandler callback function for Planes RTree search
                EventHandler<RTreeEventArgs> PlanesRTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    neighbours.Add(AgentPlanes[args.Id]);
                };

                //                   Sphere(                     future position projected on Mesh                 ,   radius      ), callback function
                PlanesRTree.Search(new Sphere(ap.P.Origin, AlignRadius), PlanesRTreeCallback);

                ap.ResetDesired();
                ap.AlignWithNeighbours(neighbours);
            }

            foreach (AgentPlane ap in AgentPlanes) ap.UpdateDirection();
        }

        /// <summary>
        /// This method prepares geometry for output
        /// </summary>
        /// <returns>a List of GH_Plane</returns>
        public List<GH_Plane> GetPlanes()
        {

            List<GH_Plane> planesOut = new List<GH_Plane>();

            foreach (AgentPlane ap in AgentPlanes) planesOut.Add(new GH_Plane(ap.P));

            return planesOut;
        }
    }

    /// <summary>
    /// The AgentPlane class represents agents with a plane to host bodies
    /// </summary>
    /// <remarks>The Plane X axis is the forward vector of the Agent</remarks>
    public class AgentPlane
    {
        public Plane P;
        public Vector3d desiredDir;

        public AgentPlane(Plane P)
        {

            this.P = P;
        }

        public void ResetDesired()
        {
            desiredDir = new Vector3d(P.XAxis);
        }

        /// <summary>
        /// Aligns forward vector with neighbour AgentPlanes
        /// </summary>
        /// <param name="neighbours">a List of AgentPlane</param>
        public void AlignWithNeighbours(List<AgentPlane> neighbours)
        {
            if (neighbours.Count == 0) return;

            Vector3d averageDir = Vector3d.Zero;

            foreach (AgentPlane neighPlane in neighbours)
            {
                averageDir += neighPlane.P.XAxis;
            }

            averageDir /= neighbours.Count;

            desiredDir += averageDir * AlignStrength;
        }

        /// <summary>
        /// This method performs alignment between X axis vectors. 
        /// </summary>
        /// <remarks>Y axis vector is found via cross-product with plane's Z axis</remarks>
        public void UpdateDirection()
        {
            Vector3d newX = P.XAxis * 0.97 + desiredDir * 0.03;
            Vector3d newY = Vector3d.CrossProduct(P.ZAxis, newX);
            P = new Plane(P.Origin, newX, newY);
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

    List<Plane> P = null;
    if (inputs[2] != null)
    {
      P = GH_DirtyCaster.CastToList<Plane>(inputs[2]);
    }
    double aligRadius = default(double);
    if (inputs[3] != null)
    {
      aligRadius = (double)(inputs[3]);
    }

    double aligStrength = default(double);
    if (inputs[4] != null)
    {
      aligStrength = (double)(inputs[4]);
    }



    //3. Declare output parameters
      object POut = null;


    //4. Invoke RunScript
    RunScript(reset, go, P, aligRadius, aligStrength, ref POut);
      
    try
    {
      //5. Assign output parameters to component...
            if (POut != null)
      {
        if (GH_Format.TreatAsCollection(POut))
        {
          IEnumerable __enum_POut = (IEnumerable)(POut);
          DA.SetDataList(1, __enum_POut);
        }
        else
        {
          if (POut is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(POut));
          }
          else
          {
            //assign direct
            DA.SetData(1, POut);
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