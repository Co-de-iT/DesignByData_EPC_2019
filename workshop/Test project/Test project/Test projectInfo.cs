using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Test_project
{
    public class Test projectInfo : GH_AssemblyInfo
  {
    public override string Name
    {
        get
        {
            return "Testproject";
        }
    }
    public override Bitmap Icon
    {
        get
        {
            //Return a 24x24 pixel bitmap to represent this GHA library.
            return null;
        }
    }
    public override string Description
    {
        get
        {
            //Return a short string describing the purpose of this GHA library.
            return "";
        }
    }
    public override Guid Id
    {
        get
        {
            return new Guid("d4dd8984-6d9b-4094-bcc1-c3c4b28756fb");
        }
    }

    public override string AuthorName
    {
        get
        {
            //Return a string identifying you or your company.
            return "";
        }
    }
    public override string AuthorContact
    {
        get
        {
            //Return a string representing your preferred contact details.
            return "";
        }
    }
}
}
