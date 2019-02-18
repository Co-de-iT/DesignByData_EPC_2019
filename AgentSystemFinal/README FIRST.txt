Before you open the files, here's some explanation and recommendations.

... EXPLANATIONS

The AgentSystemFinal is not a complete rewrite of what you know but it is largely implemented and amended, culling useless functions and introducing new ones for better output. A ffew more parameters have been added to allow further tweaking and control of fine-grain aspects.

You will find here two .gh files:

. AgentSystemFinal_basic.gh
This contains the basic rewritten version of the AgentSystem definition, improved and upgraded - for details, refer to the changelog.txt reported also in the definition itself.

. AgentSystemFinal_intermediate.gh
This adds a second simulations to the basic one:

	. this second simulation is OPTIONAL (you can entirely bypass it), and performs an alignment of the body planes with their neighbours (try it and compare before and after to understand its role). If you don't get it or think it messes up the result DO NOT USE IT FOR THE ASSIGNMENT AND STICK WITH THE BASIC - please do not waste time banging your head on this one - I'll be happy to explain the details in the code in deep AFTER you have submitted the assignment. For now, aim for the DESIGN on the basis of what you've learned in our week.

. AgentSystemFinal_full optional.gh
Like the intermediate (simulation algorithms are the same), but with some preview bells and whistles; to open this definition you will need to install first Chromodoris (https://www.food4rhino.com/app/chromodoris) and GLSL for Grasshopper (https://github.com/mcneel/ghgl/wiki/Install - nothing to download, just follow the instructions on the page)

In both files, try (from Grasshopper) View > Remote Control Panel ;)


... RECOMMENDATIONS

START WITH THE BASIC DEFINITION FIRST. If you are fluent quickly, then try the "intermediate", finally the "full_optional" one. The definitions should be sufficiently commented, but if you have any doubts, please post your questions on the Slack channel.

IMPORTANT NOTE: Use the definition you feel more confident with to get better results. If you feel comfortable and are getting good results with the one from the last day of the seminar, then by all means keep using that one! My intent is to make things better and easier, not the other way around.