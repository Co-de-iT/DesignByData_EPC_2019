using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace ws_files
{
    public class ws_filesInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "wsfiles";
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
                return new Guid("dfa65678-06fe-43b1-8470-b969cc7845b8");
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
