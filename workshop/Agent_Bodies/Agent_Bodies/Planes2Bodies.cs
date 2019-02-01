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
using System.Collections.Concurrent;
using System.Drawing;
// </Custom using> 


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance2 : GH_ScriptInstance // Script_Instance2 - name changed to avoid conflicts while writing code
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
    private void RunScript(bool reset, bool go, List<Plane> Pl, List<int> id, List<Polyline> body, ref object Bodies, ref object Planes)
    {
        // <Custom code> 

        // . . . . . . . . . . . . . . . . . . . . . . . . return on null data
        if (Pl == null || body == null) return;

        DataTree<Polyline> outBodies = new DataTree<Polyline>();
        GH_Plane[] outPlanes = new GH_Plane[Pl.Count];

        // . . . . . . . . . . . . . . . . . . . . . . . . initialize system
        if (reset || Agents == null)
        {
            ABS = new AgentBodySimulation();

            Agents = new AgentBody[Pl.Count];

            // . . . . . . . . . . . . . . . . . . . . . . . . build agents array

            for (int i = 0; i < Pl.Count; i++)
                Agents[i] = new AgentBody(Pl[i], body);

        }


        // . . . . . . . . . . . . . . . . . . . . . . . . extract geometries and data

        for (int i = 0; i < Agents.Length; i++)
            outBodies.EnsurePath(new GH_Path(i));

        List<Polyline> arms;
        for (int i = 0; i < Agents.Length; i++)
        {
            arms = Agents[i].ExtractBody();
            outBodies.AddRange(arms, new GH_Path(i));
            outPlanes[i] = new GH_Plane(Agents[i].agentPlane);
        }

        Bodies = outBodies;
        Planes = outPlanes;
        // </Custom code> 
    }

    // <Custom additional code> 

    // global variables
    public AgentBodySimulation ABS;
    public AgentBody[] Agents;

    public class AgentBodySimulation
    {
        public AgentBody[] Agents;

        public AgentBodySimulation(List<Plane> Pl, List<Polyline> body)
        {
            Agents = new AgentBody[Pl.Count];

            // . . . . . . . . . . . . . . . . . . . . . . . . build agents array
            for (int i = 0; i < Pl.Count; i++)
                Agents[i] = new AgentBody(Pl[i], body);
        }

        public AgentBodySimulation() { }
    }

    public class AgentBody
    {
        public Plane agentPlane;
        public Body agentBody;


        public AgentBody(Plane agentPlane, List<Polyline> polylines)
        {
            this.agentPlane = agentPlane;
            agentBody = new Body(polylines);
            OrientBody(Plane.WorldXY, this.agentPlane);
        }

        public void OrientBody(Plane oldPlane, Plane newPlane)
        {
            var x = Transform.PlaneToPlane(oldPlane, newPlane);

            for (int i = 0; i < agentBody.Arms.Count; i++)
                agentBody.Arms[i].Transform(x);
        }

        public List<Polyline> ExtractBody()
        {
            return agentBody.Arms;
        }

    }

    // . . . . . . . . . . . . . . . body class
    public class Body
    {
        public List<Polyline> Arms;
        public List<Point3d> Tips;
        public Point3d O;
        public Body(List<Polyline> polylines)
        {
            // ATTENTION - typical reference type mistake - shallow vs deep copies
            // this is a shallow copy, so it references always the same PolyLine list!
            //Arms = new List<Polyline>(polylines); 

            // this makes a deep copy - CORRECT
            Arms = new List<Polyline>();
            foreach (Polyline p in polylines)
                Arms.Add(p.Duplicate());

            O = Arms[0][0];

            Tips = new List<Point3d>();
            foreach (Polyline arm in Arms)
                Tips.Add(arm[arm.Count - 1]);
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
        this.doc = this.RhinoDocument;

        //2. Assign input parameters
        List<Plane> Pl = null;
        if (inputs[0] != null)
        {
            Pl = GH_DirtyCaster.CastToList<Plane>(inputs[0]);
        }
        List<int> id = null;
        if (inputs[1] != null)
        {
            id = GH_DirtyCaster.CastToList<int>(inputs[1]);
        }
        List<Polyline> body = null;
        if (inputs[2] != null)
        {
            body = GH_DirtyCaster.CastToList<Polyline>(inputs[2]);
        }


        //3. Declare output parameters
        object A = null;


        //4. Invoke RunScript
        RunScript(Pl, id, body, ref A);

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