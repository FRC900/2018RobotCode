// The function of this script is to pull data from Ryan's quintic spline graph thingy.
// This runs via mozilla rhino.
	
document.addEventListener("DOMContentLoaded", function(event)
{ 
	var elt = document.getElementById('calculator');
	var calculator = Desmos.GraphingCalculator(elt);
	//var def_vals
	loadGraph(calculator);
	console.log(calculator.getExpressions());
	// must create helper expressions for each value you want to save!
	var n_splines = calculator.HelperExpression({latex: 'n_{splines}'})
	var py = calculator.HelperExpression({latex: 'p_{y}'});
	var py1 = calculator.HelperExpression({latex: 'p_{y1}'});
	var py2 = calculator.HelperExpression({latex: 'p_{y2}'});
	var py3 = calculator.HelperExpression({latex: 'p_{y3}'});
	var py4 = calculator.HelperExpression({latex: 'p_{y4}'});
	var py5 = calculator.HelperExpression({latex: 'p_{y5}'});
	var py6 = calculator.HelperExpression({latex: 'p_{y6}'});
	var py7 = calculator.HelperExpression({latex: 'p_{y7}'});
	var py8 = calculator.HelperExpression({latex: 'p_{y8}'});
	var px = calculator.HelperExpression({latex: 'p_{x}'});
	var px1 = calculator.HelperExpression({latex: 'p_{x1}'});
	var px2 = calculator.HelperExpression({latex: 'p_{x2}'});
	var px3 = calculator.HelperExpression({latex: 'p_{x3}'});
	var px4 = calculator.HelperExpression({latex: 'p_{x4}'});
	var px5 = calculator.HelperExpression({latex: 'p_{x5}'});
	var px6 = calculator.HelperExpression({latex: 'p_{x6}'});
	var px7 = calculator.HelperExpression({latex: 'p_{x7}'});
	var px8 = calculator.HelperExpression({latex: 'p_{x8}'});


	
	var vyl = calculator.HelperExpression({latex: 'v_{yl}'});
	var vyl1 = calculator.HelperExpression({latex: 'v_{yl1}'});
	var vyl2 = calculator.HelperExpression({latex: 'v_{yl2}'});
	var vyl3 = calculator.HelperExpression({latex: 'v_{yl3}'});
	var vyl4 = calculator.HelperExpression({latex: 'v_{yl4}'});
	var vyl5 = calculator.HelperExpression({latex: 'v_{yl5}'});
	var vyl6 = calculator.HelperExpression({latex: 'v_{yl6}'});
	var vyl7 = calculator.HelperExpression({latex: 'v_{yl7}'});
	var vyl8 = calculator.HelperExpression({latex: 'v_{yl8}'});
	var vxl = calculator.HelperExpression({latex: 'v_{xl}'});
	var vxl1 = calculator.HelperExpression({latex: 'v_{xl1}'});
	var vxl2 = calculator.HelperExpression({latex: 'v_{xl2}'});
	var vxl3 = calculator.HelperExpression({latex: 'v_{xl3}'});
	var vxl4 = calculator.HelperExpression({latex: 'v_{xl4}'});
	var vxl5 = calculator.HelperExpression({latex: 'v_{xl5}'});
	var vxl6 = calculator.HelperExpression({latex: 'v_{xl6}'});
	var vxl7 = calculator.HelperExpression({latex: 'v_{xl7}'});
	var vxl8 = calculator.HelperExpression({latex: 'v_{xl8}'});

	
	var ayl = calculator.HelperExpression({latex: 'a_{yl}'});
	var ayl1 = calculator.HelperExpression({latex: 'a_{yl1}'});
	var ayl2 = calculator.HelperExpression({latex: 'a_{yl2}'});
	var ayl3 = calculator.HelperExpression({latex: 'a_{yl3}'});
	var ayl4 = calculator.HelperExpression({latex: 'a_{yl4}'});
	var ayl5 = calculator.HelperExpression({latex: 'a_{yl5}'});
	var ayl6 = calculator.HelperExpression({latex: 'a_{yl6}'});
	var ayl7 = calculator.HelperExpression({latex: 'a_{yl7}'});
	var ayl8 = calculator.HelperExpression({latex: 'a_{yl8}'});
	var axl = calculator.HelperExpression({latex: 'a_{xl}'});
	var axl1 = calculator.HelperExpression({latex: 'a_{xl1}'});
	var axl2 = calculator.HelperExpression({latex: 'a_{xl2}'});
	var axl3 = calculator.HelperExpression({latex: 'a_{xl3}'});
	var axl4 = calculator.HelperExpression({latex: 'a_{xl4}'});
	var axl5 = calculator.HelperExpression({latex: 'a_{xl5}'});
	var axl6 = calculator.HelperExpression({latex: 'a_{xl6}'});
	var axl7 = calculator.HelperExpression({latex: 'a_{xl7}'});
	var axl8 = calculator.HelperExpression({latex: 'a_{xl8}'});
	
	var Ay = calculator.HelperExpression({latex: 'A_{y}'});
	var Ay1 = calculator.HelperExpression({latex: 'A_{y1}'});
	var Ay2 = calculator.HelperExpression({latex: 'A_{y2}'});
	var Ay3 = calculator.HelperExpression({latex: 'A_{y3}'});
	var Ay4 = calculator.HelperExpression({latex: 'A_{y4}'});
	var Ay5 = calculator.HelperExpression({latex: 'A_{y5}'});
	var Ay6 = calculator.HelperExpression({latex: 'A_{y6}'});
	var Ay7 = calculator.HelperExpression({latex: 'A_{y7}'});
	var Ax = calculator.HelperExpression({latex: 'A_{x}'});
	var Ax1 = calculator.HelperExpression({latex: 'A_{x1}'});
	var Ax2 = calculator.HelperExpression({latex: 'A_{x2}'});
	var Ax3 = calculator.HelperExpression({latex: 'A_{x3}'});
	var Ax4 = calculator.HelperExpression({latex: 'A_{x4}'});
	var Ax5 = calculator.HelperExpression({latex: 'A_{x5}'});
	var Ax6 = calculator.HelperExpression({latex: 'A_{x6}'});
	var Ax7 = calculator.HelperExpression({latex: 'A_{x7}'});


	var By = calculator.HelperExpression({latex: 'B_{y}'});
	var By1 = calculator.HelperExpression({latex: 'B_{y1}'});
	var By2 = calculator.HelperExpression({latex: 'B_{y2}'});
	var By3 = calculator.HelperExpression({latex: 'B_{y3}'});
	var By4 = calculator.HelperExpression({latex: 'B_{y4}'});
	var By5 = calculator.HelperExpression({latex: 'B_{y5}'});
	var By6 = calculator.HelperExpression({latex: 'B_{y6}'});
	var By7 = calculator.HelperExpression({latex: 'B_{y7}'});
	var Bx = calculator.HelperExpression({latex: 'B_{x}'});
	var Bx1 = calculator.HelperExpression({latex: 'B_{x1}'});
	var Bx2 = calculator.HelperExpression({latex: 'B_{x2}'});
	var Bx3 = calculator.HelperExpression({latex: 'B_{x3}'});
	var Bx4 = calculator.HelperExpression({latex: 'B_{x4}'});
	var Bx5 = calculator.HelperExpression({latex: 'B_{x5}'});
	var Bx6 = calculator.HelperExpression({latex: 'B_{x6}'});
	var Bx7 = calculator.HelperExpression({latex: 'B_{x7}'});


	var Cy = calculator.HelperExpression({latex: 'C_{y}'});
	var Cy1 = calculator.HelperExpression({latex: 'C_{y1}'});
	var Cy2 = calculator.HelperExpression({latex: 'C_{y2}'});
	var Cy3 = calculator.HelperExpression({latex: 'C_{y3}'});
	var Cy4 = calculator.HelperExpression({latex: 'C_{y4}'});
	var Cy5 = calculator.HelperExpression({latex: 'C_{y5}'});
	var Cy6 = calculator.HelperExpression({latex: 'C_{y6}'});
	var Cy7 = calculator.HelperExpression({latex: 'C_{y7}'});
	var Cx = calculator.HelperExpression({latex: 'C_{x}'});
	var Cx1 = calculator.HelperExpression({latex: 'C_{x1}'});
	var Cx2 = calculator.HelperExpression({latex: 'C_{x2}'});
	var Cx3 = calculator.HelperExpression({latex: 'C_{x3}'});
	var Cx4 = calculator.HelperExpression({latex: 'C_{x4}'});
	var Cx5 = calculator.HelperExpression({latex: 'C_{x5}'});
	var Cx6 = calculator.HelperExpression({latex: 'C_{x6}'});
	var Cx7 = calculator.HelperExpression({latex: 'C_{x7}'});


	var Dy = calculator.HelperExpression({latex: 'D_{y}'});
	var Dy1 = calculator.HelperExpression({latex: 'D_{y1}'});
	var Dy2 = calculator.HelperExpression({latex: 'D_{y2}'});
	var Dy3 = calculator.HelperExpression({latex: 'D_{y3}'});
	var Dy4 = calculator.HelperExpression({latex: 'D_{y4}'});
	var Dy5 = calculator.HelperExpression({latex: 'D_{y5}'});
	var Dy6 = calculator.HelperExpression({latex: 'D_{y6}'});
	var Dy7 = calculator.HelperExpression({latex: 'D_{y7}'});
	var Dx = calculator.HelperExpression({latex: 'D_{x}'});
	var Dx1 = calculator.HelperExpression({latex: 'D_{x1}'});
	var Dx2 = calculator.HelperExpression({latex: 'D_{x2}'});
	var Dx3 = calculator.HelperExpression({latex: 'D_{x3}'});
	var Dx4 = calculator.HelperExpression({latex: 'D_{x4}'});
	var Dx5 = calculator.HelperExpression({latex: 'D_{x5}'});
	var Dx6 = calculator.HelperExpression({latex: 'D_{x6}'});
	var Dx7 = calculator.HelperExpression({latex: 'D_{x7}'});

	var Ey = calculator.HelperExpression({latex: 'E_{y}'});
	var Ey1 = calculator.HelperExpression({latex: 'E_{y1}'});
	var Ey2 = calculator.HelperExpression({latex: 'E_{y2}'});
	var Ey3 = calculator.HelperExpression({latex: 'E_{y3}'});
	var Ey4 = calculator.HelperExpression({latex: 'E_{y4}'});
	var Ey5 = calculator.HelperExpression({latex: 'E_{y5}'});
	var Ey6 = calculator.HelperExpression({latex: 'E_{y6}'});
	var Ey7 = calculator.HelperExpression({latex: 'E_{y7}'});
	var Ex = calculator.HelperExpression({latex: 'E_{x}'});
	var Ex1 = calculator.HelperExpression({latex: 'E_{x1}'});
	var Ex2 = calculator.HelperExpression({latex: 'E_{x2}'});
	var Ex3 = calculator.HelperExpression({latex: 'E_{x3}'});
	var Ex4 = calculator.HelperExpression({latex: 'E_{x4}'});
	var Ex5 = calculator.HelperExpression({latex: 'E_{x5}'});
	var Ex6 = calculator.HelperExpression({latex: 'E_{x6}'});
	var Ex7 = calculator.HelperExpression({latex: 'E_{x7}'});


	var Fy = calculator.HelperExpression({latex: 'F_{y}'});
	var Fy1 = calculator.HelperExpression({latex: 'F_{y1}'});
	var Fy2 = calculator.HelperExpression({latex: 'F_{y2}'});
	var Fy3 = calculator.HelperExpression({latex: 'F_{y3}'});
	var Fy4 = calculator.HelperExpression({latex: 'F_{y4}'});
	var Fy5 = calculator.HelperExpression({latex: 'F_{y5}'});
	var Fy6 = calculator.HelperExpression({latex: 'F_{y6}'});
	var Fy7 = calculator.HelperExpression({latex: 'F_{y7}'});
	var Fx = calculator.HelperExpression({latex: 'F_{x}'});
	var Fx1 = calculator.HelperExpression({latex: 'F_{x1}'});
	var Fx2 = calculator.HelperExpression({latex: 'F_{x2}'});
	var Fx3 = calculator.HelperExpression({latex: 'F_{x3}'});
	var Fx4 = calculator.HelperExpression({latex: 'F_{x4}'});
	var Fx5 = calculator.HelperExpression({latex: 'F_{x5}'});
	var Fx6 = calculator.HelperExpression({latex: 'F_{x6}'});
	var Fx7 = calculator.HelperExpression({latex: 'F_{x7}'});
	var valuesToSave = [n_splines, px, px1, px2, px3, px4, px5, px6, px7, px8, 
	py, py1, py2, py3, py4, py5, py6, py7, py8, 
	vxl, vxl1, vxl2, vxl3, vxl4, vxl5, vxl6, vxl7, vxl8, 
	vyl, vyl1, vyl2, vyl3, vyl4, vyl5, vyl6, vyl7, vyl8, 
	axl, axl1, axl2, axl3, axl4, axl5, axl6, axl7, axl8, 
	ayl, ayl1, ayl2, ayl3, ayl4, ayl5, ayl6, ayl7, ayl8, 

	]; 
	




	// put helper expressions in this list to pass to save()
	document.getElementById('save_load_data').onclick = function()
	{
   		console.log("Button was clicked. Beginning Save of Load Data.");
		save(valuesToSave);
	};


	
	var coefsForYaml = [
	Ax, Ax1, Ax2, Ax3, Ax4, Ax5, Ax6, Ax7,  
	Ay, Ay1, Ay2, Ay3, Ay4, Ay5, Ay6, Ay7,  
	Bx, Bx1, Bx2, Bx3, Bx4, Bx5, Bx6, Bx7,  
	By, By1, By2, By3, By4, By5, By6, By7,  
	Cx, Cx1, Cx2, Cx3, Cx4, Cx5, Cx6, Cx7,  
	Cy, Cy1, Cy2, Cy3, Cy4, Cy5, Cy6, Cy7,  
	Dx, Dx1, Dx2, Dx3, Dx4, Dx5, Dx6, Dx7,  
	Dy, Dy1, Dy2, Dy3, Dy4, Dy5, Dy6, Dy7,  
	Ex, Ex1, Ex2, Ex3, Ex4, Ex5, Ex6, Ex7,  
	Ey, Ey1, Ey2, Ey3, Ey4, Ey5, Ey6, Ey7,  
	Fx, Fx1, Fx2, Fx3, Fx4, Fx5, Fx6, Fx7,  
	Fy, Fy1, Fy2, Fy3, Fy4, Fy5, Fy6, Fy7,  
	];

	document.getElementById('save_coef_data').onclick = function()
	{
   		console.log("Button was clicked. Beginning Save of Load Data.");
		saveC(coefsForYaml, Math.round(n_splines.numericValue), 8);
	};
    document.getElementById('load_data').onclick = function()
    {
        console.log("Button was clicked. Beginning Loading of Data.");
        handleFileSelect(calculator);	
    };
	px.observe('numericValue', function () {
  		console.log(px.numericValue);
	});
	
	Cx4.observe('numericValue', function () {
  		console.log(Cx4.numericValue);
	});
	
	n_splines.observe('numericValue', function() {
		var index_num = Math.round(n_splines.numericValue);
		console.log("index num " + index_num + " splines " + n_splines.numericValue);
	});
	
});

function save (valuesToSave)
{
	var stringSave = "";

	for (var i = 0; i < valuesToSave.length; ++i)
	{
		stringSave += valuesToSave[i].numericValue + "\n";
		console.log("Saved " + valuesToSave[i].numericValue);
	}
	writeToFile(stringSave);

}

function handleFileSelect(calculator) {
    var node = document.getElementById('data');
    console.log(node.textContent);
    var output = [];
    var text = node.textContent;
    var array = text.split("\n");
    array.splice(array.length-1)
    console.log(array);
    node.textContent = array;
    loadGraphVals(calculator, array);
}


function saveC (valuesToSave, index_num, max_spline_num)
{
	var stringSave = "[";
	for (var k = 0; k < index_num; ++k)
	{
		for(var s = 0; s < 2; s++)
		{
			if(s ==0)
			{
				stringSave += "{ x: ["
			}
			else
			{

				stringSave += "y: ["

			}
			for (var i = 0; i < 5 ; ++i)
			{
				
				stringSave += valuesToSave[k + (2*i+s) * max_spline_num].numericValue + ", ";
				console.log("Saved: " + valuesToSave[k + (2*i+s) * max_spline_num].numericValue + " at index: " + k + (2*i+s) * max_spline_num);
			}
			stringSave += valuesToSave[k + (10+s) * max_spline_num].numericValue + "], ";
			if(s == 1)
			{
				if(k != index_num -1)
				{
					stringSave += "orient: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time: 4.0 }," + "\n"; 
				}
				else
				{
				stringSave += "orient: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time: 4.0 }]"+ "\n"; 
				}
			}
		}	
	}
	writeToFileC(stringSave);

}

function writeToFile(text)
{
	var textFile = null,
	makeTextFile = function (text) {
    	var data = new Blob([text], {type: 'text/plain'});

    	// If we are replacing a previously generated file we need to
    	// manually revoke the object URL to avoid memory leaks.
    	if (textFile !== null) {
      		window.URL.revokeObjectURL(textFile);
   		}

    	textFile = window.URL.createObjectURL(data);

    	return textFile;
	};

    var link = document.getElementById('downloadlink_load');
    link.href = makeTextFile(text); //text will be one long string of all the data
    link.style.display = 'block';
};
function writeToFileC(text)
{
	var textFile = null,
	makeTextFile = function (text) {
    	var data = new Blob([text], {type: 'text/plain'});

    	// If we are replacing a previously generated file we need to
    	// manually revoke the object URL to avoid memory leaks.
    	if (textFile !== null) {
      		window.URL.revokeObjectURL(textFile);
   		}

    	textFile = window.URL.createObjectURL(data);

    	return textFile;
	};

    var link = document.getElementById('downloadlink_coef');
    link.href = makeTextFile(text); //text will be one long string of all the data
    link.style.display = 'block';
};

function loadGraphVals(calculator, vals)
{
	// FIELD
	calculator.setExpression({id:'num_splines0', latex:'n_{splines} =' + vals[0].toString()});
	calculator.setExpression({id:'scale_point_y0', latex:'y_{show} = ' +7.31746 });
	calculator.setExpression({id:'scale_point_x0', latex:'x_{show} =' + 2.17});


	
	calculator.setExpression({id:'switch_point_y0', latex:'y_{switch}=3.556-22.3\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'switch_point_x0', latex:'x_{switch}=1.4'});

	
	
	calculator.setExpression({id:'exchange_point_y0', latex:'y_{exchange}=17.5\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'exchange_point_x0', latex:'x_{exchange}=-0.9145'});


	calculator.setExpression({id:'field1', latex:'y=0\\left\\{-d_1+d_2<x<d_1-d_2\\right\\}', domain:{min:'-d_1+d_2', max:'d_1-d_2'}, color: '#000000'});
	calculator.setExpression({id:'field2', latex:'d_1=\\frac{27}{2}\\cdot12\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field3', latex:'d_2=26.69\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field4', latex:'d_3=54\\cdot12\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field5', latex:'d_4=35.34\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field6', latex:'y=d_3\\left\\{-d_1+d_2<x<d_1-d_2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field7', latex:'x=d_1\\ \\left\\{d_4<y<d_3-\\ d_4\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field8', latex:'x=-d_1\\ \\left\\{d_4<y<d_3-d_4\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field9', latex:'y=\\frac{d_4}{d_2}x-\\left(\\left(d_1-d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{d_1-d_2<x<d_1\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field10', latex:'y=\\frac{-d_4}{d_2}x+d_3+\\left(\\left(d_1-d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{d_1-d_2<x<d_1\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field11', latex:'y=\\frac{d_4}{d_2}x+d_3-\\left(\\left(-d_1+d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{-d_1<x<-d_1+d_2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field12', latex:'y=\\frac{-d_4}{d_2}x+\\left(\\left(-d_1+d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{-d_1<x<-d_1+d_2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field13', latex:'d_5=140\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field14', latex:'d_6=55.81\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field15', latex:'d_7=55.5\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field16', latex:'d_{16}=76.75\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field17', latex:'y=d_5\\left\\{-d_{16}<x<d_{16}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field18', latex:'y=d_5+d_7\\left\\{-d_{16}<x<d_{16}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field19', latex:'x=-d_{16}\\ \\left\\{d_5<y<d_5+d_7\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field20', latex:'x=d_{16}\\ \\left\\{d_5<y<d_5+d_7\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field21', latex:'d_8=71.48\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field22', latex:'d_9=24.10\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field23', latex:'x=d_1-d_8\\left\\{\\frac{d_3}{2}-d_9<y<\\frac{d_3}{2}+d_9\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field24', latex:'x=-d_1+d_8\\left\\{\\frac{d_3}{2}-d_9<y<\\frac{d_3}{2}+d_9\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field25', latex:'d_{10}=36.2\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field26', latex:'y=\\frac{d_3}{2}-d_9\\left\\{d_1-d_8-d_{10}<x<d_1-d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field27', latex:'y=\\frac{d_3}{2}-d_9\\ -\\ .45', color: '#2d70b3', style: Desmos.Styles.DOTTED});
	calculator.setExpression({id:'field28', latex:'y=\\frac{d_3}{2}+d_9\\left\\{d_1-d_8-d_{10}<x<d_1-d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field29', latex:'y=\\frac{d_3}{2}-d_9\\left\\{-d_1+d_8+d_{10}>x>-d_1+d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field30', latex:'y=\\frac{d_3}{2}+d_9\\left\\{-d_1+d_8+d_{10}>x>-d_1+d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field31', latex:'d_{11}=12\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field32', latex:'d_{12}=36\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field33', latex:'d_{13}=48\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field34', latex:'x=-d_{11}\\left\\{0<y<d_{12}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field35', latex:'x\\ =\\ -d_{11}-d_{13}\\left\\{0<y<d_{12}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field36', latex:'y=d_{12}\\left\\{-d_{11}-d_{13}<x<-d_{11}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field37', latex:'d_{14}=13\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field38', latex:'d_{15}=15.1\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field39', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot0\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field40', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot0\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field41', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot0<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot0\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field42', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot1\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field43', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot1\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field44', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot1<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot1\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field45', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot2\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field46', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot2\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field47', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot2<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field48', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot3\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field49', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot3\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field50', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot3<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot3\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field51', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot4\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field52', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot4\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field53', latex:'y=d_5+d_7+d_{14}\\left\\{\\-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot4<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot4\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field54', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot5\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field55', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot5\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field56', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot5<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot5\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field57', latex:'y=d_5+d_7+d_{14}+\\frac{28}{1}\\cdot\\frac{2.54}{100}', color: '#2d70b3', style: Desmos.Styles.DOTTED});
	calculator.setExpression({id:'field58', latex:'d_{17}=\\left(3\\cdot12+5.25\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field59', latex:'d_{18}=\\left(8\\cdot12+8\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field60', latex:'d_{19}=\\left(1\\cdot12+.75\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field61', latex:'y=\\frac{d_3}{2}-d_{17}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\left\\{\\frac{-d_{18}}{2}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}<x<\\frac{d_{18}}{2}+d_{19}+\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\right\\}', color: '#c74440', style: Desmos.Styles.DASHED});
	calculator.setExpression({id:'field62', latex:'x=\\frac{-d_{18}}{2}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\left\\{\\frac{d_3}{2}-d_{17}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}<y<\\frac{d_3}{2}\\right\\}', color: '#c74440', style: Desmos.Styles.DASHED});
	calculator.setExpression({id:'field63', latex:'x=\\frac{d_{18}}{2}+d_{19}+\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\left\\{\\frac{d_3}{2}-d_{17}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}<y<\\frac{d_3}{2}\\right\\}', color: '#c74440', style: Desmos.Styles.DASHED});




   calculator.setExpression({id:'field100', latex:'d_{20}=30\\cdot\\frac{2.54}{100}'});
   calculator.setExpression({id:'field64', latex:'x\ =\ \\left(\\frac{13}{2}+13\\right)\\cdot\\frac{2.54}{100}\\left\{d_5-13\\cdot\\frac{2.54}{100}<y<d_5\\right\\}'});
   calculator.setExpression({id:'field65', latex:'x\ =\ -\\left(13\\right)\\cdot\\frac{2.54}{100}\\left\\{d_5-2\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field66', latex:'x\ =\ \\left(13\\right)\\cdot\\frac{2.54}{100}\\left\{d_5-2\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot\\frac{2.54}{100}\\right\}'});
   calculator.setExpression({id:'field67', latex:'x\ =\ \\left(13\\right)\\cdot\\frac{2.54}{100}\\left\\{d_5-2\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field68', latex:'x\ =\ \\left(\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\left\{d_5-3\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot2\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field69', latex:'x\ =\ -\\left(\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\left\\{d_5-3\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot2\\cdot\\frac{2.54}{100}\\right\\}'});
	 
   calculator.setExpression({id:'field70', latex:'\\left(y+\\left(13\\cdot2+\\frac{13}{2}\\right)\cdot\\frac{2.54}{100}-d_5\\right)^2+\\left(x\\right)^2=d_{20}^2'});
   calculator.setExpression({id:'field64', latex:'\\left(0,\ d_5-\\left(13\\cdot2+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\right)'});
   calculator.setExpression({id:'field71', latex:'\\left(\\frac{13}{2}\\cdot\\frac{2.54}{100},\ d_5-\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\right)'});
   calculator.setExpression({id:'field72', latex:'\\left(\\frac{-13}{2}\\cdot\\frac{2.54}{100},\ d_5-\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\right)'});
   calculator.setExpression({id:'field73', latex:'\left(y+\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}-d_5\\right)^2+\\left(x+\\frac{13}{2}\\cdot\\frac{2.54}{100}\\right)^2=d_{20}^2'});
   calculator.setExpression({id:'field74', latex:'\\left(y+\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}-d_5\\right)^2+\\left(x-\\frac{13}{2}\\cdot\\frac{2.54}{100}\\right)^2=d_{20}^2'});







	// POSITIONS
	calculator.setExpression({id:'position64', latex:'x=1'});
	calculator.setExpression({id:'position64', latex:'\\left(p_x,\\ p_y\\right)', color: '#c74440',hidden: true});
	calculator.setExpression({id:'position65', latex:'p_x=' + vals[1].toString()});
	calculator.setExpression({id:'position66', latex:'p_y=' + vals[10].toString() }); //
	calculator.setExpression({id:'position67', latex:'\\left(p_{x1},\\ p_{y1}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'position68', latex:'p_{x1}=' + vals[2].toString() }); //2.226'});
	calculator.setExpression({id:'position69', latex:'p_{y1}=' + vals[11].toString() }); //7.31'});
	calculator.setExpression({id:'position70', latex:'\\left(p_{x2},\\ p_{y2}\\right)', color: '#388c46'});
	calculator.setExpression({id:'position71', latex:'p_{x2}=' + vals[3].toString() }); //1.774'});
	calculator.setExpression({id:'position73', latex:'p_{y2}=' + vals[12].toString() }); //6.25'});
	calculator.setExpression({id:'position74', latex:'\\left(p_{x3},\\ p_{y3}\\right)', color: '#fa7e19'});
	calculator.setExpression({id:'position75', latex:'p_{x3}=' + vals[4].toString() }); //0.395'});
	calculator.setExpression({id:'position76', latex:'p_{y3}=' + vals[13].toString() }); //5.744'});
	calculator.setExpression({id:'position77', latex:'\\left(p_{x4},\\ p_{y4}\\right)', color: '#6042a6'});
	calculator.setExpression({id:'position78', latex:'p_{x4}=' + vals[5].toString() }); //2.176'});
	calculator.setExpression({id:'position79', latex:'p_{y4}=' + vals[14].toString() }); //7.1'});
	calculator.setExpression({id:'position80', latex:'\\left(p_{x5},\\ p_{y5}\\right)', color: '#000000'});
	calculator.setExpression({id:'position81', latex:'p_{x5}=' + vals[6].toString() }); //1.3'});
	calculator.setExpression({id:'position82', latex:'p_{y5}=' + vals[15].toString() }); //5.723'});
	calculator.setExpression({id:'position83', latex:'\\left(p_{x6},\\ p_{y6}\\right)', color: '#c74440'});
	calculator.setExpression({id:'position84', latex:'p_{x6}=' + vals[7].toString() }); //2.24'});
	calculator.setExpression({id:'position85', latex:'p_{y6}=' + vals[16].toString() }); //7.21'});
	calculator.setExpression({id:'position86', latex:'\\left(p_{x7},\\ p_{y7}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'position87', latex:'p_{x7}=' + vals[8].toString() }); //1.83'});
	calculator.setExpression({id:'position88', latex:'p_{y7}=' + vals[17].toString() }); //5.56'});
	calculator.setExpression({id:'position89', latex:'\\left(p_{x8},\\ p_{y8}\\right)', color: '#000000'});
	calculator.setExpression({id:'position90', latex:'p_{x8}=' + vals[9].toString() }); //0'});
	calculator.setExpression({id:'position91', latex:'p_{y8}=' + vals[18].toString() }); //0'});

	







	// VELOCITIES
	calculator.setExpression({id:'velocity93', latex:'\\left(v_{xl},\\ v_{yl}\\right)', color: '#c74440'});
	calculator.setExpression({id:'velocity94', latex:'v_{xl}=' + vals[19].toString() }); //2.77'});
	calculator.setExpression({id:'velocity95', latex:'v_{yl}=' + vals[28].toString() }); //4.02'});
	calculator.setExpression({id:'velocity96', latex:'\\left(v_{xl1},\\ v_{yl1}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'velocity97', latex:'v_{xl1}=' + vals[20].toString() }); //1.86'});
	calculator.setExpression({id:'velocity98', latex:'v_{yl1}=' + vals[29].toString() }); //7.5'});
	calculator.setExpression({id:'velocity99', latex:'\\left(v_{xl2},\\ v_{yl2}\\right)', color: '#388c46'});
	calculator.setExpression({id:'velocity100', latex:'v_{xl2}=' + vals[21].toString() }); //0.46'});
	calculator.setExpression({id:'velocity101', latex:'v_{yl2}=' + vals[30].toString() }); //5.657'});
	calculator.setExpression({id:'velocity102', latex:'\\left(v_{xl3},\\ v_{yl3}\\right)', color: '#fa7e19'});
	calculator.setExpression({id:'velocity103', latex:'v_{xl3}=' + vals[22].toString() }); //0.79'});
	calculator.setExpression({id:'velocity104', latex:'v_{yl3}=' + vals[31].toString() }); //5.626'});
	calculator.setExpression({id:'velocity105', latex:'\\left(v_{xl4},\\ v_{yl4}\\right)', color: '#6042a6'});
	calculator.setExpression({id:'velocity106', latex:'v_{xl4}=' + vals[23].toString() }); //2.77'});
	calculator.setExpression({id:'velocity107', latex:'v_{yl4}=' + vals[32].toString() }); //9.073'});
	calculator.setExpression({id:'velocity108', latex:'\\left(v_{xl5},\\ v_{yl5}\\right)', color: '#000000'});
	calculator.setExpression({id:'velocity109', latex:'v_{xl5}=' + vals[24].toString() }); //-0.204'});
	calculator.setExpression({id:'velocity110', latex:'v_{yl5}=' + vals[33].toString() }); //5.475'});
	calculator.setExpression({id:'velocity111', latex:'\\left(v_{xl6},\\ v_{yl6}\\right)', color: '#c74440'});
	calculator.setExpression({id:'velocity112', latex:'v_{xl6}=' + vals[25].toString() }); //1.91'});
	calculator.setExpression({id:'velocity113', latex:'v_{yl6}=' + vals[34].toString() }); //8.45'});
	calculator.setExpression({id:'velocity114', latex:'\\left(v_{xl7},\\ v_{yl7}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'velocity115', latex:'v_{xl7}=' + vals[26].toString() }); //1.89'});
	calculator.setExpression({id:'velocity116', latex:'v_{yl7}=' + vals[35].toString() }); //4.83'});
	calculator.setExpression({id:'velocity117', latex:'\\left(v_{xl8},\\ v_{yl8}\\right)', color: '#000000'});
	calculator.setExpression({id:'velocity118', latex:'v_{xl8}=' + vals[27].toString() }); //-1.1'});
	calculator.setExpression({id:'velocity119', latex:'v_{yl8}=' + vals[36].toString() }); //-2.7'});


	
	// ACCELERATIONS
	calculator.setExpression({id:'acceleration120', latex:'\\left(a_{xl},\\ a_{yl}\\right)', color: '#c74440'});
	calculator.setExpression({id:'acceleration121', latex:'a_{xl}=' + vals[37].toString() }); //6.49'});
	calculator.setExpression({id:'acceleration122', latex:'a_{yl}=' + vals[46].toString() }); //3.72'});
	calculator.setExpression({id:'acceleration123', latex:'\\left(a_{xl1},\\ a_{yl1}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'acceleration124', latex:'a_{xl1}=' + vals[38].toString() }); //1.13'});
	calculator.setExpression({id:'acceleration125', latex:'a_{yl1}=' + vals[47].toString() }); //6.54'});
	calculator.setExpression({id:'acceleration126', latex:'\\left(a_{xl2},\\ a_{yl2}\\right)', color: '#388c46'});
	calculator.setExpression({id:'acceleration127', latex:'a_{xl2}=' + vals[39].toString() }); //-3.5'});
	calculator.setExpression({id:'acceleration128', latex:'a_{yl2}=' + vals[48].toString() }); //2.48'});
	calculator.setExpression({id:'acceleration129', latex:'\\left(a_{xl3},\\ a_{yl3}\\right)', color: '#fa7319'});
	calculator.setExpression({id:'acceleration130', latex:'a_{xl3}=' + vals[40].toString() }); //12'});
	calculator.setExpression({id:'acceleration131', latex:'a_{yl3}=' + vals[49].toString() }); //7.7'});
	calculator.setExpression({id:'acceleration132', latex:'\\left(a_{xl4},\\ a_{yl4}\\right)', color: '#6042a6'});
	calculator.setExpression({id:'acceleration133', latex:'a_{xl4}=' + vals[41].toString() }); //2.55'});
	calculator.setExpression({id:'acceleration134', latex:'a_{yl4}=' + vals[50].toString() }); //6.29'});
	calculator.setExpression({id:'acceleration135', latex:'\\left(a_{xl5},\\ a_{yl5}\\right)', color: '#000000'});
	calculator.setExpression({id:'acceleration136', latex:'a_{xl5}=' + vals[42].toString() }); //0.797'});
	calculator.setExpression({id:'acceleration137', latex:'a_{yl5}=' + vals[51].toString() }); //6.034'});
	calculator.setExpression({id:'acceleration138', latex:'\\left(a_{xl6},\\ a_{yl6}\\right)', color: '#c74440'});
	calculator.setExpression({id:'acceleration139', latex:'a_{xl6}=' + vals[43].toString() }); //2.87'});
	calculator.setExpression({id:'acceleration140', latex:'a_{yl6}=' + vals[52].toString() }); //6.91'});
	calculator.setExpression({id:'acceleration141', latex:'\\left(a_{xl7},\\ a_{yl7}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'acceleration142', latex:'a_{xl7}=' + vals[44].toString() }); //2.32'});
	calculator.setExpression({id:'acceleration143', latex:'a_{yl7}=' + vals[53].toString() }); //5.08'});
	calculator.setExpression({id:'acceleration144', latex:'\\left(a_{xl8},\\ a_{yl8}\\right)', color: '#000000'});
	calculator.setExpression({id:'acceleration145', latex:'a_{xl8}=' + vals[45].toString() }); //-0.7'});
	calculator.setExpression({id:'acceleration146', latex:'a_{yl8}=' + vals[54].toString() }); //-1.64'});




	// SPLINES
	calculator.setExpression({id:'spline147', latex:'A_x\\ =\\ \\frac{1}{2}\\left(\\left(a_{x1}-a_x\\right)-6\\left(v_x+v_{x1}\\right)+12\\left(p_{x1}-p_x\\right)\\right)'});
	calculator.setExpression({id:'spline148', latex:'B_x\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x1}\\left(2\\cdot\\left(1\\right)+3\\cdot\\left(0\\right)\\right)-a_x\\left(3\\cdot\\left(1\\right)+2\\cdot\\left(0\\right)\\right)\\right)\\ +2\\ \\left(v_x\\left(8\\cdot\\left(1\\right)+7\\cdot\\left(0\\right)\\right)+v_{x1}\\left(7\\cdot\\left(1\\right)+8\\cdot\\left(0\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x1}-\\ p_x\\right)\\left(0+1\\right)\\right)'});
	calculator.setExpression({id:'spline149', latex:'C_x\\ =\\ \\frac{1}{2}\\left(\\left(a_{x1}\\left(3\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)-a_x\\left(\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)\\right)-4\\left(v_x\\left(2\\left(0\\right)^2+10\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)+v_{x1}\\left(3\\left(0\\right)^2+10\\left(1\\right)\\cdot\\left(0\\right)+2\\left(1\\right)^2\\right)\\right)+20\\left(p_{x1}-p_x\\right)\\left(\\left(0\\right)^2+4\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline150', latex:'D_x=\\frac{a_x}{2}-10A_x\\left(0\\right)^3-6B_x\\left(0\\right)^2-3C_x\\cdot\\left(0\\right)'});
	calculator.setExpression({id:'spline151', latex:'E_x=v_x-5A_x\\left(0\\right)^4-4B_x\\left(0\\right)^3-3C_x\\left(0\\right)^2-2D_x\\left(0\\right)'});
	calculator.setExpression({id:'spline152', latex:'F_x=p_x-A_x\\left(0\\right)^5-B_x\\left(0\\right)^4-C_x\\left(0\\right)^3-D_x\\left(0\\right)^2-E_x\\left(0\\right)'});
	calculator.setExpression({id:'spline153', latex:'A_y\\ =\\ \\frac{1}{2}\\left(\\left(a_{y1}-a_y\\right)-6\\left(v_y+v_{y1}\\right)+12\\left(p_{y1}-p_y\\right)\\right)'});
	calculator.setExpression({id:'spline154', latex:'B_y\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y1}\\left(2\\cdot\\left(1\\right)+3\\cdot\\left(0\\right)\\right)-a_y\\left(3\\cdot\\left(1\\right)+2\\cdot\\left(0\\right)\\right)\\right)\\ +2\\ \\left(v_y\\left(8\\cdot\\left(1\\right)+7\\cdot\\left(0\\right)\\right)+v_{y1}\\left(7\\cdot\\left(1\\right)+8\\cdot\\left(0\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y1}-\\ p_y\\right)\\left(0+1\\right)\\right)'});
	calculator.setExpression({id:'spline155', latex:'C_y\\ =\\ \\frac{1}{2}\\left(\\left(a_{y1}\\left(3\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)-a_y\\left(\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)\\right)-4\\left(v_y\\left(2\\left(0\\right)^2+10\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)+v_{y1}\\left(3\\left(0\\right)^2+10\\left(1\\right)\\left(0\\right)+2\\left(1\\right)^2\\right)\\right)+20\\left(p_{y1}-p_y\\right)\\left(\\left(0\\right)^2+4\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline156', latex:'D_y=\\frac{a_y}{2}-10A_y\\left(0\\right)^3-6B_y\\left(0\\right)^2-3C_y\\cdot\\left(0\\right)'});
	calculator.setExpression({id:'spline157', latex:'E_y=v_y-5A_y\\left(0\\right)^4-4B_y\\left(0\\right)^3-3C_y\\left(0\\right)^2-2D_y\\left(0\\right)'});
	calculator.setExpression({id:'spline159', latex:'F_y=p_y-A_y\\left(0\\right)^5-B_y\\left(0\\right)^4-C_y\\left(0\\right)^3-D_y\\left(0\\right)^2-E_y\\left(0\\right)'});
	calculator.setExpression({id:'spline160', latex:'f\\left(x\\right)\\ =\\ A_xx^5+B_xx^4+C_xx^3+D_xx^2+E_xx+F_x', hidden: true});
	calculator.setExpression({id:'spline161', latex:'g\\left(x\\right)\\ =\\ A_yx^5+B_yx^4+C_yx^3+D_yx^2+E_yx+F_y', hidden: true});
	calculator.setExpression({id:'spline162', latex:'A_{x1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x2}-a_{x1}\\right)-6\\left(v_{x1}+v_{x2}\\right)+12\\left(p_{x2}-p_{x1}\\right)\\right)'});
	calculator.setExpression({id:'spline163', latex:'B_{x1}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x2}\\left(2\\cdot\\left(2\\right)+3\\cdot\\left(1\\right)\\right)-a_{x1}\\left(3\\cdot\\left(2\\right)+2\\cdot\\left(1\\right)\\right)\\right)\\ +2\\ \\left(v_{x1}\\left(8\\cdot\\left(2\\right)+7\\cdot\\left(1\\right)\\right)+v_{x2}\\left(7\\cdot\\left(2\\right)+8\\cdot\\left(1\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x2}-\\ p_{x1}\\right)\\left(1+2\\right)\\right)'});
	calculator.setExpression({id:'spline164', latex:'C_{x1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x2}\\left(3\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)-a_{x1}\\left(\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)\\right)-4\\left(v_{x1}\\left(2\\left(1\\right)^2+10\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)+v_{x2}\\left(3\\left(1\\right)^2+10\\left(2\\right)\\cdot\\left(1\\right)+2\\left(2\\right)^2\\right)\\right)+20\\left(p_{x2}-p_{x1}\\right)\\left(\\left(1\\right)^2+4\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline165', latex:'D_{x1}=\\frac{a_{x1}}{2}-10A_{x1}\\left(1\\right)^3-6B_{x1}\\left(1\\right)^2-3C_{x1}\\cdot\\left(1\\right)'});
	calculator.setExpression({id:'spline166', latex:'E_{x1}=v_{x1}-5A_{x1}\\left(1\\right)^4-4B_{x1}\\left(1\\right)^3-3C_{x1}\\left(1\\right)^2-2D_{x1}\\left(1\\right)'});
	calculator.setExpression({id:'spline167', latex:'F_{x1}=p_{x1}-A_{x1}\\left(1\\right)^5-B_{x1}\\left(1\\right)^4-C_{x1}\\left(1\\right)^3-D_{x1}\\left(1\\right)^2-E_{x1}\\left(1\\right)'});
	calculator.setExpression({id:'spline168', latex:'A_{y1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y2}-a_{y1}\\right)-6\\left(v_{y1}+v_{y2}\\right)+12\\left(p_{y2}-p_{y1}\\right)\\right)'});
	calculator.setExpression({id:'spline169', latex:'B_{y1}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y2}\\left(2\\cdot\\left(2\\right)+3\\cdot\\left(1\\right)\\right)-a_{y1}\\left(3\\cdot\\left(2\\right)+2\\cdot\\left(1\\right)\\right)\\right)\\ +2\\ \\left(v_{y1}\\left(8\\cdot\\left(2\\right)+7\\cdot\\left(1\\right)\\right)+v_{y2}\\left(7\\cdot\\left(2\\right)+8\\cdot\\left(1\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y2}-\\ p_{y1}\\right)\\left(1+2\\right)\\right)'});
	calculator.setExpression({id:'spline170', latex:'C_{y1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y2}\\left(3\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)-a_{y1}\\left(\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)\\right)-4\\left(v_{y1}\\left(2\\left(1\\right)^2+10\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)+v_{y2}\\left(3\\left(1\\right)^2+10\\left(2\\right)\\left(1\\right)+2\\left(2\\right)^2\\right)\\right)+20\\left(p_{y2}-p_{y1}\\right)\\left(\\left(1\\right)^2+4\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline171', latex:'D_{y1}=\\frac{a_{y1}}{2}-10A_{y1}\\left(1\\right)^3-6B_{y1}\\left(1\\right)^2-3C_{y1}\\cdot\\left(1\\right)'});
	calculator.setExpression({id:'spline172', latex:'E_{y1}=v_{y1}-5A_{y1}\\left(1\\right)^4-4B_{y1}\\left(1\\right)^3-3C_{y1}\\left(1\\right)^2-2D_{y1}\\left(1\\right)'});
	calculator.setExpression({id:'spline173', latex:'F_{y1}=p_{y1}-A_{y1}\\left(1\\right)^5-B_{y1}\\left(1\\right)^4-C_{y1}\\left(1\\right)^3-D_{y1}\\left(1\\right)^2-E_{y1}\\left(1\\right)'});
	calculator.setExpression({id:'spline174', latex:'f_1\\left(x\\right)\\ =\\ A_{x1}x^5+B_{x1}x^4+C_{x1}x^3+D_{x1}x^2+E_{x1}x+F_{x1}', hidden: true});
	calculator.setExpression({id:'spline175', latex:'g_1\\left(x\\right)\\ =\\ A_{y1}x^5+B_{y1}x^4+C_{y1}x^3+D_{y1}x^2+E_{y1}x+F_{y1}', hidden: true});
	calculator.setExpression({id:'spline176', latex:'A_{x2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x3}-a_{x2}\\right)-6\\left(v_{x2}+v_{x3}\\right)+12\\left(p_{x3}-p_{x2}\\right)\\right)'});
	calculator.setExpression({id:'spline177', latex:'B_{x2}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x3}\\left(2\\cdot\\left(3\\right)+3\\cdot\\left(2\\right)\\right)-a_{x2}\\left(3\\cdot\\left(3\\right)+2\\cdot\\left(2\\right)\\right)\\right)\\ +2\\ \\left(v_{x2}\\left(8\\cdot\\left(3\\right)+7\\cdot\\left(2\\right)\\right)+v_{x3}\\left(7\\cdot\\left(3\\right)+8\\cdot\\left(2\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x3}-\\ p_{x2}\\right)\\left(2+3\\right)\\right)'});
	calculator.setExpression({id:'spline178', latex:'C_{x2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x3}\\left(3\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)-a_{x2}\\left(\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)\\right)-4\\left(v_{x2}\\left(2\\left(2\\right)^2+10\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)+v_{x3}\\left(3\\left(2\\right)^2+10\\left(3\\right)\\cdot\\left(2\\right)+2\\left(3\\right)^2\\right)\\right)+20\\left(p_{x3}-p_{x2}\\right)\\left(\\left(2\\right)^2+4\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline179', latex:'D_{x2}=\\frac{a_{x2}}{2}-10A_{x2}\\left(2\\right)^3-6B_{x2}\\left(2\\right)^2-3C_{x2}\\cdot\\left(2\\right)'});
	calculator.setExpression({id:'spline180', latex:'E_{x2}=v_{x2}-5A_{x2}\\left(2\\right)^4-4B_{x2}\\left(2\\right)^3-3C_{x2}\\left(2\\right)^2-2D_{x2}\\left(2\\right)'});
	calculator.setExpression({id:'spline181', latex:'F_{x2}=p_{x2}-A_{x2}\\left(2\\right)^5-B_{x2}\\left(2\\right)^4-C_{x2}\\left(2\\right)^3-D_{x2}\\left(2\\right)^2-E_{x2}\\left(2\\right)'});
	calculator.setExpression({id:'spline182', latex:'A_{y2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y3}-a_{y2}\\right)-6\\left(v_{y2}+v_{y3}\\right)+12\\left(p_{y3}-p_{y2}\\right)\\right)'});
	calculator.setExpression({id:'spline183', latex:'B_{y2}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y3}\\left(2\\cdot\\left(3\\right)+3\\cdot\\left(2\\right)\\right)-a_{y2}\\left(3\\cdot\\left(3\\right)+2\\cdot\\left(2\\right)\\right)\\right)\\ +2\\ \\left(v_{y2}\\left(8\\cdot\\left(3\\right)+7\\cdot\\left(2\\right)\\right)+v_{y3}\\left(7\\cdot\\left(3\\right)+8\\cdot\\left(2\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y3}-\\ p_{y2}\\right)\\left(2+3\\right)\\right)'});
	calculator.setExpression({id:'spline184', latex:'C_{y2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y3}\\left(3\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)-a_{y2}\\left(\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)\\right)-4\\left(v_{y2}\\left(2\\left(2\\right)^2+10\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)+v_{y3}\\left(3\\left(2\\right)^2+10\\left(3\\right)\\left(2\\right)+2\\left(3\\right)^2\\right)\\right)+20\\left(p_{y3}-p_{y2}\\right)\\left(\\left(2\\right)^2+4\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline185', latex:'D_{y2}=\\frac{a_{y2}}{2}-10A_{y2}\\left(2\\right)^3-6B_{y2}\\left(2\\right)^2-3C_{y2}\\cdot\\left(2\\right)'});
	calculator.setExpression({id:'spline186', latex:'E_{y2}=v_{y2}-5A_{y2}\\left(2\\right)^4-4B_{y2}\\left(2\\right)^3-3C_{y2}\\left(2\\right)^2-2D_{y2}\\left(2\\right)'});
	calculator.setExpression({id:'spline187', latex:'F_{y2}=p_{y2}-A_{y2}\\left(2\\right)^5-B_{y2}\\left(2\\right)^4-C_{y2}\\left(2\\right)^3-D_{y2}\\left(2\\right)^2-E_{y2}\\left(2\\right)'});
	calculator.setExpression({id:'spline188', latex:'f_2\\left(x\\right)\\ =\\ A_{x2}x^5+B_{x2}x^4+C_{x2}x^3+D_{x2}x^2+E_{x2}x+F_{x2}', hidden: true});
	calculator.setExpression({id:'spline189', latex:'g_2\\left(x\\right)\\ =\\ A_{y2}x^5+B_{y2}x^4+C_{y2}x^3+D_{y2}x^2+E_{y2}x+F_{y2}', hidden: true});
	calculator.setExpression({id:'spline190', latex:'A_{x3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x4}-a_{x3}\\right)-6\\left(v_{x3}+v_{x4}\\right)+12\\left(p_{x4}-p_{x3}\\right)\\right)'});
	calculator.setExpression({id:'spline191', latex:'B_{x3}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x4}\\left(2\\cdot\\left(4\\right)+3\\cdot\\left(3\\right)\\right)-a_{x3}\\left(3\\cdot\\left(4\\right)+2\\cdot\\left(3\\right)\\right)\\right)\\ +2\\ \\left(v_{x3}\\left(8\\cdot\\left(4\\right)+7\\cdot\\left(3\\right)\\right)+v_{x4}\\left(7\\cdot\\left(4\\right)+8\\cdot\\left(3\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x4}-\\ p_{x3}\\right)\\left(3+4\\right)\\right)'});
	calculator.setExpression({id:'spline192', latex:'C_{x3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x4}\\left(3\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)-a_{x3}\\left(\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)\\right)-4\\left(v_{x3}\\left(2\\left(3\\right)^2+10\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)+v_{x4}\\left(3\\left(3\\right)^2+10\\left(4\\right)\\cdot\\left(3\\right)+2\\left(4\\right)^2\\right)\\right)+20\\left(p_{x4}-p_{x3}\\right)\\left(\\left(3\\right)^2+4\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline193', latex:'D_{x3}=\\frac{a_{x3}}{2}-10A_{x3}\\left(3\\right)^3-6B_{x3}\\left(3\\right)^2-3C_{x3}\\cdot\\left(3\\right)'});
	calculator.setExpression({id:'spline194', latex:'E_{x3}=v_{x3}-5A_{x3}\\left(3\\right)^4-4B_{x3}\\left(3\\right)^3-3C_{x3}\\left(3\\right)^2-2D_{x3}\\left(3\\right)'});
	calculator.setExpression({id:'spline195', latex:'F_{x3}=p_{x3}-A_{x3}\\left(3\\right)^5-B_{x3}\\left(3\\right)^4-C_{x3}\\left(3\\right)^3-D_{x3}\\left(3\\right)^2-E_{x3}\\left(3\\right)'});
	calculator.setExpression({id:'spline196', latex:'A_{y3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y4}-a_{y3}\\right)-6\\left(v_{y3}+v_{y4}\\right)+12\\left(p_{y4}-p_{y3}\\right)\\right)'});
	calculator.setExpression({id:'spline197', latex:'B_{y3}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y4}\\left(2\\cdot\\left(4\\right)+3\\cdot\\left(3\\right)\\right)-a_{y3}\\left(3\\cdot\\left(4\\right)+2\\cdot\\left(3\\right)\\right)\\right)\\ +2\\ \\left(v_{y3}\\left(8\\cdot\\left(4\\right)+7\\cdot\\left(3\\right)\\right)+v_{y4}\\left(7\\cdot\\left(4\\right)+8\\cdot\\left(3\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y4}-\\ p_{y3}\\right)\\left(3+4\\right)\\right)'});
	calculator.setExpression({id:'spline198', latex:'C_{y3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y4}\\left(3\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)-a_{y3}\\left(\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)\\right)-4\\left(v_{y3}\\left(2\\left(3\\right)^2+10\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)+v_{y4}\\left(3\\left(3\\right)^2+10\\left(4\\right)\\left(3\\right)+2\\left(4\\right)^2\\right)\\right)+20\\left(p_{y4}-p_{y3}\\right)\\left(\\left(3\\right)^2+4\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline199', latex:'D_{y3}=\\frac{a_{y3}}{2}-10A_{y3}\\left(3\\right)^3-6B_{y3}\\left(3\\right)^2-3C_{y3}\\cdot\\left(3\\right)'});
	calculator.setExpression({id:'spline200', latex:'E_{y3}=v_{y3}-5A_{y3}\\left(3\\right)^4-4B_{y3}\\left(3\\right)^3-3C_{y3}\\left(3\\right)^2-2D_{y3}\\left(3\\right)'});
	calculator.setExpression({id:'spline201', latex:'F_{y3}=p_{y3}-A_{y3}\\left(3\\right)^5-B_{y3}\\left(3\\right)^4-C_{y3}\\left(3\\right)^3-D_{y3}\\left(3\\right)^2-E_{y3}\\left(3\\right)'});
	calculator.setExpression({id:'spline202', latex:'f_3\\left(x\\right)\\ =\\ A_{x3}x^5+B_{x3}x^4+C_{x3}x^3+D_{x3}x^2+E_{x3}x+F_{x3}', hidden: true});
	calculator.setExpression({id:'spline203', latex:'g_3\\left(x\\right)\\ =\\ A_{y3}x^5+B_{y3}x^4+C_{y3}x^3+D_{y3}x^2+E_{y3}x+F_{y3}', hidden: true});
	calculator.setExpression({id:'spline204', latex:'A_{x4}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x5}-a_{x4}\\right)-6\\left(v_{x4}+v_{x5}\\right)+12\\left(p_{x5}-p_{x4}\\right)\\right)'});
	calculator.setExpression({id:'spline205', latex:'B_{x4}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x5}\\left(2\\cdot\\left(5\\right)+3\\cdot\\left(4\\right)\\right)-a_{x4}\\left(3\\cdot\\left(5\\right)+2\\cdot\\left(4\\right)\\right)\\right)\\ +2\\ \\left(v_{x4}\\left(8\\cdot\\left(5\\right)+7\\cdot\\left(4\\right)\\right)+v_{x5}\\left(7\\cdot\\left(5\\right)+8\\cdot\\left(4\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x5}-\\ p_{x4}\\right)\\left(5+4\\right)\\right)'});
	calculator.setExpression({id:'spline206', latex:'C_{x4}=\\ \\frac{1}{2}\\left(\\left(a_{x5}\\left(3\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)-a_{x4}\\left(\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)\\right)-4\\left(v_{x4}\\left(2\\left(4\\right)^2+10\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)+v_{x5}\\left(3\\left(4\\right)^2+10\\left(5\\right)\\cdot\\left(4\\right)+2\\left(5\\right)^2\\right)\\right)+20\\left(p_{x5}-p_{x4}\\right)\\left(\\left(4\\right)^2+4\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline207', latex:'D_{x4}=\\frac{a_{x4}}{2}-10A_{x4}\\left(4\\right)^3-6B_{x4}\\left(4\\right)^2-3C_{x4}\\cdot\\left(4\\right)'});
	calculator.setExpression({id:'spline208', latex:'E_{x4}=v_{x4}-5A_{x4}\\left(4\\right)^4-4B_{x4}\\left(4\\right)^3-3C_{x4}\\left(4\\right)^2-2D_{x4}\\left(4\\right)'});
	calculator.setExpression({id:'spline209', latex:'F_{x4}=p_{x4}-A_{x4}\\left(4\\right)^5-B_{x4}\\left(4\\right)^4-C_{x4}\\left(4\\right)^3-D_{x4}\\left(4\\right)^2-E_{x4}\\left(4\\right)'});
	calculator.setExpression({id:'spline210', latex:'A_{y4}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y5}-a_{y4}\\right)-6\\left(v_{y4}+v_{y5}\\right)+12\\left(p_{y5}-p_{y4}\\right)\\right)'});
	calculator.setExpression({id:'spline211', latex:'B_{y4}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y5}\\left(2\\cdot\\left(5\\right)+3\\cdot\\left(4\\right)\\right)-a_{y4}\\left(3\\cdot\\left(5\\right)+2\\cdot\\left(4\\right)\\right)\\right)\\ +2\\ \\left(v_{y4}\\left(8\\cdot\\left(5\\right)+7\\cdot\\left(4\\right)\\right)+v_{y5}\\left(7\\cdot\\left(5\\right)+8\\cdot\\left(4\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y5}-\\ p_{y4}\\right)\\left(5+4\\right)\\right)'});
	calculator.setExpression({id:'spline212', latex:'C_{y4}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y5}\\left(3\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)-a_{y4}\\left(\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)\\right)-4\\left(v_{y4}\\left(2\\left(4\\right)^2+10\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)+v_{y5}\\left(3\\left(4\\right)^2+10\\left(5\\right)\\left(4\\right)+2\\left(5\\right)^2\\right)\\right)+20\\left(p_{y5}-p_{y4}\\right)\\left(\\left(4\\right)^2+4\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline213', latex:'D_{y4}=\\frac{a_{y4}}{2}-10A_{y4}\\left(4\\right)^3-6B_{y4}\\left(4\\right)^2-3C_{y4}\\cdot\\left(4\\right)'});
	calculator.setExpression({id:'spline214', latex:'E_{y4}=v_{y4}-5A_{y4}\\left(4\\right)^4-4B_{y4}\\left(4\\right)^3-3C_{y4}\\left(4\\right)^2-2D_{y4}\\left(4\\right)'});
	calculator.setExpression({id:'spline215', latex:'F_{y4}=p_{y4}-A_{y4}\\left(4\\right)^5-B_{y4}\\left(4\\right)^4-C_{y4}\\left(4\\right)^3-D_{y4}\\left(4\\right)^2-E_{y4}\\left(4\\right)'});
	calculator.setExpression({id:'spline216', latex:'f_4\\left(x\\right)\\ =\\ A_{x4}x^5+B_{x4}x^4+C_{x4}x^3+D_{x4}x^2+E_{x4}x+F_{x4}', hidden: true});
	calculator.setExpression({id:'spline217', latex:'g_4\\left(x\\right)\\ =\\ A_{y4}x^5+B_{y4}x^4+C_{y4}x^3+D_{y4}x^2+E_{y4}x+F_{y4}', hidden: true});
	calculator.setExpression({id:'spline218', latex:'A_{x5}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x6}-a_{x5}\\right)-6\\left(v_{x5}+v_{x6}\\right)+12\\left(p_{x6}-p_{x5}\\right)\\right)'});
	calculator.setExpression({id:'spline219', latex:'B_{x5}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x6}\\left(2\\cdot\\left(6\\right)+3\\cdot\\left(5\\right)\\right)-a_{x5}\\left(3\\cdot\\left(6\\right)+2\\cdot\\left(5\\right)\\right)\\right)\\ +2\\ \\left(v_{x5}\\left(8\\cdot\\left(6\\right)+7\\cdot\\left(5\\right)\\right)+v_{x6}\\left(7\\cdot\\left(6\\right)+8\\cdot\\left(5\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x6}-\\ p_{x5}\\right)\\left(6+5\\right)\\right)'});
	calculator.setExpression({id:'spline220', latex:'C_{x5}=\\ \\frac{1}{2}\\left(\\left(a_{x6}\\left(3\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)-a_{x5}\\left(\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)\\right)-4\\left(v_{x5}\\left(2\\left(5\\right)^2+10\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)+v_{x6}\\left(3\\left(5\\right)^2+10\\left(6\\right)\\cdot\\left(5\\right)+2\\left(6\\right)^2\\right)\\right)+20\\left(p_{x6}-p_{x5}\\right)\\left(\\left(5\\right)^2+4\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline221', latex:'D_{x5}=\\frac{a_{x5}}{2}-10A_{x5}\\left(5\\right)^3-6B_{x5}\\left(5\\right)^2-3C_{x5}\\cdot\\left(5\\right)'});
	calculator.setExpression({id:'spline222', latex:'E_{x5}=v_{x5}-5A_{x5}\\left(5\\right)^4-4B_{x5}\\left(5\\right)^3-3C_{x5}\\left(5\\right)^2-2D_{x5}\\left(5\\right)'});
	calculator.setExpression({id:'spline223', latex:'F_{x5}=p_{x5}-A_{x5}\\left(5\\right)^5-B_{x5}\\left(5\\right)^4-C_{x5}\\left(5\\right)^3-D_{x5}\\left(5\\right)^2-E_{x5}\\left(5\\right)'});
	calculator.setExpression({id:'spline224', latex:'A_{y5}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y6}-a_{y5}\\right)-6\\left(v_{y5}+v_{y6}\\right)+12\\left(p_{y6}-p_{y5}\\right)\\right)'});
	calculator.setExpression({id:'spline225', latex:'B_{y5}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y6}\\left(2\\cdot\\left(6\\right)+3\\cdot\\left(5\\right)\\right)-a_{y5}\\left(3\\cdot\\left(6\\right)+2\\cdot\\left(5\\right)\\right)\\right)\\ +2\\ \\left(v_{y5}\\left(8\\cdot\\left(6\\right)+7\\cdot\\left(5\\right)\\right)+v_{y6}\\left(7\\cdot\\left(6\\right)+8\\cdot\\left(5\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y6}-\\ p_{y5}\\right)\\left(6+5\\right)\\right)'});
	calculator.setExpression({id:'spline226', latex:'C_{y5}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y6}\\left(3\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)-a_{y5}\\left(\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)\\right)-4\\left(v_{y5}\\left(2\\left(5\\right)^2+10\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)+v_{y6}\\left(3\\left(5\\right)^2+10\\left(6\\right)\\left(5\\right)+2\\left(6\\right)^2\\right)\\right)+20\\left(p_{y6}-p_{y5}\\right)\\left(\\left(5\\right)^2+4\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline227', latex:'D_{y5}=\\frac{a_{y5}}{2}-10A_{y5}\\left(5\\right)^3-6B_{y5}\\left(5\\right)^2-3C_{y5}\\cdot\\left(5\\right)'});
	calculator.setExpression({id:'spline228', latex:'E_{y5}=v_{y5}-5A_{y5}\\left(5\\right)^4-4B_{y5}\\left(5\\right)^3-3C_{y5}\\left(5\\right)^2-2D_{y5}\\left(5\\right)'});
	calculator.setExpression({id:'spline229', latex:'F_{y5}=p_{y5}-A_{y5}\\left(5\\right)^5-B_{y5}\\left(5\\right)^4-C_{y5}\\left(5\\right)^3-D_{y5}\\left(5\\right)^2-E_{y5}\\left(5\\right)'});
	calculator.setExpression({id:'spline230', latex:'f_5\\left(x\\right)\\ =\\ A_{x5}x^5+B_{x5}x^4+C_{x5}x^3+D_{x5}x^2+E_{x5}x+F_{x5}', hidden: true});
	calculator.setExpression({id:'spline231', latex:'g_5\\left(x\\right)\\ =\\ A_{y5}x^5+B_{y5}x^4+C_{y5}x^3+D_{y5}x^2+E_{y5}x+F_{y5}', hidden: true});
	calculator.setExpression({id:'spline232', latex:'A_{x6}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x7}-a_{x6}\\right)-6\\left(v_{x6}+v_{x7}\\right)+12\\left(p_{x7}-p_{x6}\\right)\\right)'});
	calculator.setExpression({id:'spline233', latex:'B_{x6}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x7}\\left(2\\cdot\\left(7\\right)+3\\cdot\\left(6\\right)\\right)-a_{x6}\\left(3\\cdot\\left(7\\right)+2\\cdot\\left(6\\right)\\right)\\right)\\ +2\\ \\left(v_{x6}\\left(8\\cdot\\left(7\\right)+7\\cdot\\left(6\\right)\\right)+v_{x7}\\left(7\\cdot\\left(7\\right)+8\\cdot\\left(6\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x7}-\\ p_{x6}\\right)\\left(7+6\\right)\\right)'});
	calculator.setExpression({id:'spline234', latex:'C_{x6}=\\ \\frac{1}{2}\\left(\\left(a_{x7}\\left(3\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)-a_{x6}\\left(\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)\\right)-4\\left(v_{x6}\\left(2\\left(6\\right)^2+10\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)+v_{x7}\\left(3\\left(6\\right)^2+10\\left(7\\right)\\cdot\\left(6\\right)+2\\left(7\\right)^2\\right)\\right)+20\\left(p_{x7}-p_{x6}\\right)\\left(\\left(6\\right)^2+4\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline235', latex:'D_{x6}=\\frac{a_{x6}}{2}-10A_{x6}\\left(6\\right)^3-6B_{x6}\\left(6\\right)^2-3C_{x6}\\cdot\\left(6\\right)'});
	calculator.setExpression({id:'spline236', latex:'E_{x6}=v_{x6}-5A_{x6}\\left(6\\right)^4-4B_{x6}\\left(6\\right)^3-3C_{x6}\\left(6\\right)^2-2D_{x6}\\left(6\\right)'});
	calculator.setExpression({id:'spline237', latex:'F_{x6}=p_{x6}-A_{x6}\\left(6\\right)^5-B_{x6}\\left(6\\right)^4-C_{x6}\\left(6\\right)^3-D_{x6}\\left(6\\right)^2-E_{x6}\\left(6\\right)'});
	calculator.setExpression({id:'spline238', latex:'A_{y6}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y7}-a_{y6}\\right)-6\\left(v_{y6}+v_{y7}\\right)+12\\left(p_{y7}-p_{y6}\\right)\\right)'});
	calculator.setExpression({id:'spline239', latex:'B_{y6}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y7}\\left(2\\cdot\\left(7\\right)+3\\cdot\\left(6\\right)\\right)-a_{y6}\\left(3\\cdot\\left(7\\right)+2\\cdot\\left(6\\right)\\right)\\right)\\ +2\\ \\left(v_{y6}\\left(8\\cdot\\left(7\\right)+7\\cdot\\left(6\\right)\\right)+v_{y7}\\left(7\\cdot\\left(7\\right)+8\\cdot\\left(6\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y7}-\\ p_{y6}\\right)\\left(7+6\\right)\\right)'});
	calculator.setExpression({id:'spline240', latex:'C_{y6}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y7}\\left(3\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)-a_{y6}\\left(\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)\\right)-4\\left(v_{y6}\\left(2\\left(6\\right)^2+10\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)+v_{y7}\\left(3\\left(6\\right)^2+10\\left(7\\right)\\left(6\\right)+2\\left(7\\right)^2\\right)\\right)+20\\left(p_{y7}-p_{y6}\\right)\\left(\\left(6\\right)^2+4\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline241', latex:'D_{y6}=\\frac{a_{y6}}{2}-10A_{y6}\\left(6\\right)^3-6B_{y6}\\left(6\\right)^2-3C_{y6}\\cdot\\left(6\\right)'});
	calculator.setExpression({id:'spline242', latex:'E_{y6}=v_{y6}-5A_{y6}\\left(6\\right)^4-4B_{y6}\\left(6\\right)^3-3C_{y6}\\left(6\\right)^2-2D_{y6}\\left(6\\right)'});
	calculator.setExpression({id:'spline243', latex:'F_{y6}=p_{y6}-A_{y6}\\left(6\\right)^5-B_{y6}\\left(6\\right)^4-C_{y6}\\left(6\\right)^3-D_{y6}\\left(6\\right)^2-E_{y6}\\left(6\\right)'});
	calculator.setExpression({id:'spline244', latex:'f_6\\left(x\\right)\\ =\\ A_{x6}x^5+B_{x6}x^4+C_{x6}x^3+D_{x6}x^2+E_{x6}x+F_{x6}', hidden: true});
	calculator.setExpression({id:'spline245', latex:'g_6\\left(x\\right)\\ =\\ A_{y6}x^5+B_{y6}x^4+C_{y6}x^3+D_{y6}x^2+E_{y6}x+F_{y6}', hidden: true});
	calculator.setExpression({id:'spline246', latex:'A_{x7}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x8}-a_{x7}\\right)-6\\left(v_{x7}+v_{x8}\\right)+12\\left(p_{x8}-p_{x7}\\right)\\right)'});
	calculator.setExpression({id:'spline247', latex:'B_{x7}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x8}\\left(2\\cdot\\left(8\\right)+3\\cdot\\left(7\\right)\\right)-a_{x7}\\left(3\\cdot\\left(8\\right)+2\\cdot\\left(7\\right)\\right)\\right)\\ +2\\ \\left(v_{x7}\\left(8\\cdot\\left(8\\right)+7\\cdot\\left(7\\right)\\right)+v_{x8}\\left(7\\cdot\\left(8\\right)+8\\cdot\\left(7\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x8}-\\ p_{x7}\\right)\\left(8+7\\right)\\right)'});
	calculator.setExpression({id:'spline248', latex:'C_{x7}=\\ \\frac{1}{2}\\left(\\left(a_{x8}\\left(3\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)-a_{x7}\\left(\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)\\right)-4\\left(v_{x7}\\left(2\\left(7\\right)^2+10\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)+v_{x8}\\left(3\\left(7\\right)^2+10\\left(8\\right)\\cdot\\left(7\\right)+2\\left(8\\right)^2\\right)\\right)+20\\left(p_{x8}-p_{x7}\\right)\\left(\\left(7\\right)^2+4\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline249', latex:'D_{x7}=\\frac{a_{x7}}{2}-10A_{x7}\\left(7\\right)^3-6B_{x7}\\left(7\\right)^2-3C_{x7}\\cdot\\left(7\\right)'});
	calculator.setExpression({id:'spline250', latex:'E_{x7}=v_{x7}-5A_{x7}\\left(7\\right)^4-4B_{x7}\\left(7\\right)^3-3C_{x7}\\left(7\\right)^2-2D_{x7}\\left(7\\right)'});
	calculator.setExpression({id:'spline251', latex:'F_{x7}=p_{x7}-A_{x7}\\left(7\\right)^5-B_{x7}\\left(7\\right)^4-C_{x7}\\left(7\\right)^3-D_{x7}\\left(7\\right)^2-E_{x7}\\left(7\\right)'});
	calculator.setExpression({id:'spline252', latex:'A_{y7}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y8}-a_{y7}\\right)-6\\left(v_{y7}+v_{y8}\\right)+12\\left(p_{y8}-p_{y7}\\right)\\right)'});
	calculator.setExpression({id:'spline253', latex:'12\\left(p_{y8}-p_{y7}\\right)'});
	calculator.setExpression({id:'spline254', latex:'B_{y7}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y8}\\left(2\\cdot\\left(8\\right)+3\\cdot\\left(7\\right)\\right)-a_{y7}\\left(3\\cdot\\left(8\\right)+2\\cdot\\left(7\\right)\\right)\\right)\\ +2\\ \\left(v_{y7}\\left(8\\cdot\\left(8\\right)+7\\cdot\\left(7\\right)\\right)+v_{y8}\\left(7\\cdot\\left(8\\right)+8\\cdot\\left(7\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y8}-\\ p_{y7}\\right)\\left(8+7\\right)\\right)'});
	calculator.setExpression({id:'spline255', latex:'C_{y7}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y8}\\left(3\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)-a_{y7}\\left(\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)\\right)-4\\left(v_{y7}\\left(2\\left(7\\right)^2+10\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)+v_{y8}\\left(3\\left(7\\right)^2+10\\left(8\\right)\\left(7\\right)+2\\left(8\\right)^2\\right)\\right)+20\\left(p_{y8}-p_{y7}\\right)\\left(\\left(7\\right)^2+4\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline256', latex:'D_{y7}=\\frac{a_{y7}}{2}-10A_{y7}\\left(7\\right)^3-6B_{y7}\\left(7\\right)^2-3C_{y7}\\cdot\\left(7\\right)'});
	calculator.setExpression({id:'spline257', latex:'E_{y7}=v_{y7}-5A_{y7}\\left(7\\right)^4-4B_{y7}\\left(7\\right)^3-3C_{y7}\\left(7\\right)^2-2D_{y7}\\left(7\\right)'});
	calculator.setExpression({id:'spline258', latex:'F_{y7}=p_{y7}-A_{y7}\\left(7\\right)^5-B_{y7}\\left(7\\right)^4-C_{y7}\\left(7\\right)^3-D_{y7}\\left(7\\right)^2-E_{y7}\\left(7\\right)'});
	calculator.setExpression({id:'spline259', latex:'f_7\\left(x\\right)\\ =\\ A_{x7}x^5+B_{x7}x^4+C_{x7}x^3+D_{x7}x^2+E_{x7}x+F_{x7}', hidden: true});
	calculator.setExpression({id:'spline260', latex:'g_7\\left(x\\right)\\ =\\ A_{y7}x^5+B_{y7}x^4+C_{y7}x^3+D_{y7}x^2+E_{y7}x+F_{y7}', hidden: true});


	






	// OUT
	calculator.setExpression({id:'out261', latex:'v_x\\ =\\left(\\ v_{xl}-p_x\\right)'});
	calculator.setExpression({id:'out262', latex:'v_y\\ =\\ \\left(\\ v_{yl}-p_y\\right)'});
	calculator.setExpression({id:'out263', latex:'v_{x1}\\ =\\left(\\ v_{xl1}-p_{x1}\\right)'});
	calculator.setExpression({id:'out264', latex:'v_{y1}\\ =\\ \\left(v_{yl1}-p_{y1}\\right)'});
	calculator.setExpression({id:'out265', latex:'v_{x2}\\ =\\left(\\ v_{xl2}-p_{x2}\\right)'});
	calculator.setExpression({id:'out266', latex:'v_{y2}\\ =\\ v_{yl2}-p_{y2}'});
	calculator.setExpression({id:'out267', latex:'v_{x3}\\ =\\left(\\ v_{xl3}-p_{x3}\\right)'});
	calculator.setExpression({id:'out268', latex:'v_{y3}=\\ \\left(v_{yl3}-p_{y3}\\right)'});
	calculator.setExpression({id:'out269', latex:'v_{x4}\\ =\\left(\\ v_{xl4}-p_{x4}\\right)'});
	calculator.setExpression({id:'out270', latex:'v_{y4}=\\ \\left(v_{yl4}-p_{y4}\\right)'});
	calculator.setExpression({id:'out271', latex:'v_{x5}\\ =\\left(\\ v_{xl5}-p_{x5}\\right)'});
	calculator.setExpression({id:'out272', latex:'v_{y5}=\\ \\left(v_{yl5}-p_{y5}\\right)'});
	calculator.setExpression({id:'out273', latex:'v_{x6}\\ =\\left(\\ v_{xl6}-p_{x6}\\right)'});
	calculator.setExpression({id:'out274', latex:'v_{y6}=\\ \\left(v_{yl6}-p_{y6}\\right)'});
	calculator.setExpression({id:'out275', latex:'v_{x7}\\ =\\left(\\ v_{xl7}-p_{x7}\\right)'});
	calculator.setExpression({id:'out276', latex:'v_{y7}=\\ \\left(v_{yl7}-p_{y7}\\right)'});
	calculator.setExpression({id:'out277', latex:'v_{x8}\\ =\\left(\\ v_{xl8}-p_{x8}\\right)'});
	calculator.setExpression({id:'out278', latex:'v_{y8}=\\ \\left(v_{yl8}-p_{y8}\\right)'});
	calculator.setExpression({id:'out279', latex:'a_x\\ =a_{xl}-p_x-v_x'});
	calculator.setExpression({id:'out280', latex:'a_y\\ =\\ a_{yl}-p_y-v_y'});
	calculator.setExpression({id:'out281', latex:'\\left(a_xt^2+\\left(v_x-2\\cdot\\left(0\\right)\\cdot a_x\\right)t+\\left(p_x-\\left(0\\right)\\cdot\\left(v_x-2\\cdot\\left(0\\right)\\cdot a_x\\right)-\\left(0\\right)^2\\cdot a_x\\right),\\ a_yt^2+\\left(v_y-2\\cdot\\left(0\\right)\\cdot a_y\\right)t+\\left(p_y-\\left(0\\right)\\cdot\\left(v_y-2\\cdot\\left(0\\right)\\cdot a_y\\right)-\\left(0\\right)^2\\cdot a_y\\right)\\right)', hidden: true, domain:{min: -1, max: 1}});
	calculator.setExpression({id:'out282', latex:'a_{x1}\\ =a_{xl1}-p_{x1}-v_{x1}'});
	calculator.setExpression({id:'out283', latex:'a_{y1}\\ =\\ a_{yl1}-p_{y1}-v_{y1}'});
	calculator.setExpression({id:'out284', latex:'\\left(a_{x1}t^2+\\left(v_{x1}-2\\cdot\\left(1\\right)\\cdot a_{x1}\\right)t+\\left(p_{x1}-\\left(1\\right)\\cdot\\left(v_{x1}-2\\cdot\\left(1\\right)\\cdot a_{x1}\\right)-\\left(1\\right)^2\\cdot a_{x1}\\right),\\ a_{y1}t^2+\\left(v_{y1}-2\\cdot\\left(1\\right)\\cdot a_{y1}\\right)t+\\left(p_{y1}-\\left(1\\right)\\cdot\\left(v_{y1}-2\\cdot\\left(1\\right)\\cdot a_{y1}\\right)-\\left(1\\right)^2\\cdot a_{y1}\\right)\\right)', hidden: true, domain:{min: 0, max: 2}});
	calculator.setExpression({id:'out285', latex:'a_{x2}\\ =a_{xl2}-p_{x2}-v_{x2}'});
	calculator.setExpression({id:'out286', latex:'a_{y2}\\ =\\ a_{yl2}-p_{y2}-v_{y2}'});
	calculator.setExpression({id:'out287', latex:'\\left(a_{x2}t^2+\\left(v_{x2}-2\\cdot\\left(2\\right)\\cdot a_{x2}\\right)t+\\left(p_{x2}-\\left(2\\right)\\cdot\\left(v_{x2}-2\\cdot\\left(2\\right)\\cdot a_{x2}\\right)-\\left(2\\right)^2\\cdot a_{x2}\\right),\\ a_{y2}t^2+\\left(v_{y2}-2\\cdot\\left(2\\right)\\cdot a_{y2}\\right)t+\\left(p_{y2}-\\left(2\\right)\\cdot\\left(v_{y2}-2\\cdot\\left(2\\right)\\cdot a_{y2}\\right)-\\left(2\\right)^2\\cdot a_{y2}\\right)\\right)', hidden: true, domain:{min: 1, max: 3}});
	calculator.setExpression({id:'out288', latex:'a_{x3}\\ =a_{xl3}-p_{x3}-v_{x3}'});
	calculator.setExpression({id:'out289', latex:'a_{y3}\\ =\\ a_{yl3}-p_{y3}-v_{y3}'});
	calculator.setExpression({id:'out290', latex:'\\left(a_{x3}t^2+\\left(v_{x3}-2\\cdot\\left(3\\right)\\cdot a_{x3}\\right)t+\\left(p_{x3}-\\left(3\\right)\\cdot\\left(v_{x3}-2\\cdot\\left(3\\right)\\cdot a_{x3}\\right)-\\left(3\\right)^2\\cdot a_{x3}\\right),\\ a_{y3}t^2+\\left(v_{y3}-2\\cdot\\left(3\\right)\\cdot a_{y3}\\right)t+\\left(p_{y3}-\\left(3\\right)\\cdot\\left(v_{y3}-2\\cdot\\left(3\\right)\\cdot a_{y3}\\right)-\\left(3\\right)^2\\cdot a_{y3}\\right)\\right)', hidden: true, domain:{min: 2, max: 4}});
	calculator.setExpression({id:'out291', latex:'a_{x4}\\ =a_{xl4}-p_{x4}-v_{x4}'});
	calculator.setExpression({id:'out292', latex:'a_{y4}\\ =\\ a_{yl4}-p_{y4}-v_{y4}'});
	calculator.setExpression({id:'out293', latex:'\\left(a_{x4}t^2+\\left(v_{x4}-2\\cdot\\left(4\\right)\\cdot a_{x4}\\right)t+\\left(p_{x4}-\\left(4\\right)\\cdot\\left(v_{x4}-2\\cdot\\left(4\\right)\\cdot a_{x4}\\right)-\\left(4\\right)^2\\cdot a_{x4}\\right),\\ a_{y4}t^2+\\left(v_{y4}-2\\cdot\\left(4\\right)\\cdot a_{y4}\\right)t+\\left(p_{y4}-\\left(4\\right)\\cdot\\left(v_{y4}-2\\cdot\\left(4\\right)\\cdot a_{y4}\\right)-\\left(4\\right)^2\\cdot a_{y4}\\right)\\right)', hidden: true, domain:{min: 3, max: 5}});
	calculator.setExpression({id:'out294', latex:'a_{x5}\\ =a_{xl5}-p_{x5}-v_{x5}'});
	calculator.setExpression({id:'out295', latex:'a_{y5}\\ =\\ a_{yl5}-p_{y5}-v_{y5}'});
	calculator.setExpression({id:'out296', latex:'\\left(a_{x5}t^2+\\left(v_{x5}-2\\cdot\\left(5\\right)\\cdot a_{x5}\\right)t+\\left(p_{x5}-\\left(5\\right)\\cdot\\left(v_{x5}-2\\cdot\\left(5\\right)\\cdot a_{x5}\\right)-\\left(5\\right)^2\\cdot a_{x5}\\right),\\ a_{y5}t^2+\\left(v_{y5}-2\\cdot\\left(5\\right)\\cdot a_{y5}\\right)t+\\left(p_{y5}-\\left(5\\right)\\cdot\\left(v_{y5}-2\\cdot\\left(5\\right)\\cdot a_{y5}\\right)-\\left(5\\right)^2\\cdot a_{y5}\\right)\\right)', hidden: true, domain:{min: 4, max: 6}});
	calculator.setExpression({id:'out297', latex:'a_{x6}\\ =a_{xl6}-p_{x6}-v_{x6}'});
	calculator.setExpression({id:'out298', latex:'a_{y6}\\ =\\ a_{yl6}-p_{y6}-v_{y6}'});
	calculator.setExpression({id:'out299', latex:'\\left(a_{x6}t^2+\\left(v_{x6}-2\\cdot\\left(6\\right)\\cdot a_{x6}\\right)t+\\left(p_{x6}-\\left(6\\right)\\cdot\\left(v_{x6}-2\\cdot\\left(6\\right)\\cdot a_{x6}\\right)-\\left(6\\right)^2\\cdot a_{x6}\\right),\\ a_{y6}t^2+\\left(v_{y6}-2\\cdot\\left(6\\right)\\cdot a_{y6}\\right)t+\\left(p_{y6}-\\left(6\\right)\\cdot\\left(v_{y6}-2\\cdot\\left(6\\right)\\cdot a_{y6}\\right)-\\left(6\\right)^2\\cdot a_{y6}\\right)\\right)', hidden: true, domain:{min: 5, max: 7}});
	calculator.setExpression({id:'out300', latex:'a_{x7}\\ =a_{xl7}-p_{x7}-v_{x7}'});
	calculator.setExpression({id:'out301', latex:'a_{y7}\\ =\\ a_{yl7}-p_{y7}-v_{y7}'});
	calculator.setExpression({id:'out302', latex:'\\left(a_{x7}t^2+\\left(v_{x7}-2\\cdot\\left(7\\right)\\cdot a_{x7}\\right)t+\\left(p_{x7}-\\left(7\\right)\\cdot\\left(v_{x7}-2\\cdot\\left(7\\right)\\cdot a_{x7}\\right)-\\left(7\\right)^2\\cdot a_{x7}\\right),\\ a_{y7}t^2+\\left(v_{y7}-2\\cdot\\left(7\\right)\\cdot a_{y7}\\right)t+\\left(p_{y7}-\\left(7\\right)\\cdot\\left(v_{y7}-2\\cdot\\left(7\\right)\\cdot a_{y7}\\right)-\\left(7\\right)^2\\cdot a_{y7}\\right)\\right)', hidden: true, domain:{min: 6, max: 8}});
	calculator.setExpression({id:'out303', latex:'a_{x8}\\ =a_{xl8}-p_{x8}-v_{x8}'});
	calculator.setExpression({id:'out304', latex:'a_{y8}\\ =\\ a_{yl8}-p_{y8}-v_{y8}'});
	calculator.setExpression({id:'out305', latex:'\\left(a_{x8}t^2+\\left(v_{x8}-2\\cdot\\left(8\\right)\\cdot a_{x8}\\right)t+\\left(p_{x8}-\\left(8\\right)\\cdot\\left(v_{x8}-2\\cdot\\left(8\\right)\\cdot a_{x8}\\right)-\\left(8\\right)^2\\cdot a_{x8}\\right),\\ a_{y8}t^2+\\left(v_{y8}-2\\cdot\\left(8\\right)\\cdot a_{y8}\\right)t+\\left(p_{y8}-\\left(8\\right)\\cdot\\left(v_{y8}-2\\cdot\\left(8\\right)\\cdot a_{y8}\\right)-\\left(8\\right)^2\\cdot a_{y8}\\right)\\right)', hidden: true, domain:{min: 7, max: 9}});
	
	// FUNCTIONS
	
	if(vals[0] < 1)
	{ 
		calculator.setExpression({id:'func306', latex:'\\left(f\\left(t\\right),\\ g\\left(t\\right)\\right)', color: '#c74440', domain:{min: 0, max: 1}, hidden: true});
	}
	else
	{
		calculator.setExpression({id:'func306', latex:'\\left(f\\left(t\\right),\\ g\\left(t\\right)\\right)', color: '#c74440', domain:{min: 0, max: 1}});

	}
	if(vals[0] < 2)
	{ 
		calculator.setExpression({id:'func307', latex:'\\left(f_1\\left(t\\right),\\ g_1\\left(t\\right)\\right)', domain:{min: 1, max: 2}, hidden: true  });
	}
	else
	{
		calculator.setExpression({id:'func307', latex:'\\left(f_1\\left(t\\right),\\ g_1\\left(t\\right)\\right)', domain:{min: 1, max: 2}});
	}


	if(vals[0] < 3)
	{ 
		calculator.setExpression({id:'func308', latex:'\\left(f_2\\left(t\\right),\\ g_2\\left(t\\right)\\right)', domain:{min: 2, max: 3},hidden: true });
	}
	else
	{
		calculator.setExpression({id:'func308', latex:'\\left(f_2\\left(t\\right),\\ g_2\\left(t\\right)\\right)', domain:{min: 2, max: 3}});

	}




	if(vals[0] < 4)
	{ 
		calculator.setExpression({id:'func309', latex:'\\left(f_3\\left(t\\right),\\ g_3\\left(t\\right)\\right)', domain:{min: 3, max: 4}, hidden: true});
	}
	else
	{

		calculator.setExpression({id:'func309', latex:'\\left(f_3\\left(t\\right),\\ g_3\\left(t\\right)\\right)', domain:{min: 3, max: 4}});
	}

	if(vals[0] < 5)
	{
		calculator.setExpression({id:'func310', latex:'\\left(f_4\\left(t\\right),\\ g_4\\left(t\\right)\\right)', domain:{min: 4, max: 5}, hidden: true});

	}
	else
	{
		calculator.setExpression({id:'func310', latex:'\\left(f_4\\left(t\\right),\\ g_4\\left(t\\right)\\right)', domain:{min: 4, max: 5}});

	}
	if(vals[0] < 6)
	{
	calculator.setExpression({id:'func311', latex:'\\left(f_5\\left(t\\right),\\ g_5\\left(t\\right)\\right)', domain:{min: 5, max: 6}, hidden: true});

	}
	else
	{
	calculator.setExpression({id:'func311', latex:'\\left(f_5\\left(t\\right),\\ g_5\\left(t\\right)\\right)', domain:{min: 5, max: 6}});

	}
	if(vals[0] < 7)
	{
	calculator.setExpression({id:'func312', latex:'\\left(f_6\\left(t\\right),\\ g_6\\left(t\\right)\\right)', color: '#c74440', domain:{min: 6, max: 7}, hidden: true});

	}
	else
	{
	calculator.setExpression({id:'func312', latex:'\\left(f_6\\left(t\\right),\\ g_6\\left(t\\right)\\right)', color: '#c74440', domain:{min: 6, max: 7}});

	}
	if(vals[0] < 8)
	{
	calculator.setExpression({id:'func315', latex:'\\left(f_7\\left(t\\right),\\ g_7\\left(t\\right)\\right)', color: '#000000', domain:{min: 7, max: 8}, hidden: true});

	}
	else
	{
	calculator.setExpression({id:'func315', latex:'\\left(f_7\\left(t\\right),\\ g_7\\left(t\\right)\\right)', color: '#000000', domain:{min: 7, max: 8}});

	}


}

function loadGraph(calculator)
{
	// FIELD
	calculator.setExpression({id:'num_splines0', latex:'n_{splines} = 5'});
	calculator.setExpression({id:'scale_point_y0', latex:'y_{scale} = ' +7.31746 });
	calculator.setExpression({id:'scale_point_x0', latex:'x_{scale} =' + 2.17});
	
	calculator.setExpression({id:'switch_point_y0', latex:'y_{switch}=3.556-22.3\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'switch_point_x0', latex:'x_{switch}=1.4'});

	
	
	calculator.setExpression({id:'exchange_point_y0', latex:'y_{exchange}=17.5\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'exchange_point_x0', latex:'x_{exchange}=-0.9145'});







	calculator.setExpression({id:'field1', latex:'y=0\\left\\{-d_1+d_2<x<d_1-d_2\\right\\}', domain:{min:'-d_1+d_2', max:'d_1-d_2'}, color: '#000000'});
	calculator.setExpression({id:'field2', latex:'d_1=\\frac{27}{2}\\cdot12\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field3', latex:'d_2=26.69\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field4', latex:'d_3=54\\cdot12\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field5', latex:'d_4=35.34\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field6', latex:'y=d_3\\left\\{-d_1+d_2<x<d_1-d_2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field7', latex:'x=d_1\\ \\left\\{d_4<y<d_3-\\ d_4\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field8', latex:'x=-d_1\\ \\left\\{d_4<y<d_3-d_4\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field9', latex:'y=\\frac{d_4}{d_2}x-\\left(\\left(d_1-d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{d_1-d_2<x<d_1\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field10', latex:'y=\\frac{-d_4}{d_2}x+d_3+\\left(\\left(d_1-d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{d_1-d_2<x<d_1\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field11', latex:'y=\\frac{d_4}{d_2}x+d_3-\\left(\\left(-d_1+d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{-d_1<x<-d_1+d_2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field12', latex:'y=\\frac{-d_4}{d_2}x+\\left(\\left(-d_1+d_2\\right)\\cdot\\frac{d_4}{d_2}\\right)\\left\\{-d_1<x<-d_1+d_2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field13', latex:'d_5=140\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field14', latex:'d_6=55.81\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field15', latex:'d_7=55.5\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field16', latex:'d_{16}=76.75\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field17', latex:'y=d_5\\left\\{-d_{16}<x<d_{16}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field18', latex:'y=d_5+d_7\\left\\{-d_{16}<x<d_{16}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field19', latex:'x=-d_{16}\\ \\left\\{d_5<y<d_5+d_7\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field20', latex:'x=d_{16}\\ \\left\\{d_5<y<d_5+d_7\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field21', latex:'d_8=71.48\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field22', latex:'d_9=24.10\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field23', latex:'x=d_1-d_8\\left\\{\\frac{d_3}{2}-d_9<y<\\frac{d_3}{2}+d_9\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field24', latex:'x=-d_1+d_8\\left\\{\\frac{d_3}{2}-d_9<y<\\frac{d_3}{2}+d_9\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field25', latex:'d_{10}=36.2\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field26', latex:'y=\\frac{d_3}{2}-d_9\\left\\{d_1-d_8-d_{10}<x<d_1-d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field27', latex:'y=\\frac{d_3}{2}-d_9\\ -\\ .3', color: '#2d70b3', style: Desmos.Styles.DOTTED});
	calculator.setExpression({id:'field28', latex:'y=\\frac{d_3}{2}+d_9\\left\\{d_1-d_8-d_{10}<x<d_1-d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field29', latex:'y=\\frac{d_3}{2}-d_9\\left\\{-d_1+d_8+d_{10}>x>-d_1+d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field30', latex:'y=\\frac{d_3}{2}+d_9\\left\\{-d_1+d_8+d_{10}>x>-d_1+d_8\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field31', latex:'d_{11}=12\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field32', latex:'d_{12}=36\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field33', latex:'d_{13}=48\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field34', latex:'x=-d_{11}\\left\\{0<y<d_{12}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field35', latex:'x\\ =\\ -d_{11}-d_{13}\\left\\{0<y<d_{12}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field36', latex:'y=d_{12}\\left\\{-d_{11}-d_{13}<x<-d_{11}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field37', latex:'d_{14}=13\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field38', latex:'d_{15}=15.1\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field39', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot0\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field40', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot0\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field41', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot0<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot0\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field42', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot1\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field43', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot1\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field44', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot1<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot1\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field45', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot2\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field46', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot2\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field47', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot2<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot2\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field48', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot3\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field49', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot3\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field50', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot3<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot3\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field51', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot4\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field52', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot4\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field53', latex:'y=d_5+d_7+d_{14}\\left\\{\\-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot4<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot4\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field54', latex:'x=-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot5\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field55', latex:'x=-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot5\\left\\{d_5+d_7<y<d_5+d_7+d_{14}\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field56', latex:'y=d_5+d_7+d_{14}\\left\\{-d_{16}+\\left(d_{15}+d_{14}\\right)\\cdot5<x<-d_{16}+d_{14}+\\left(d_{15}+d_{14}\\right)\\cdot5\\right\\}', color: '#000000'});
	calculator.setExpression({id:'field57', latex:'y=d_5+d_7+d_{14}+\\frac{28}{1}\\cdot\\frac{2.54}{100}', color: '#2d70b3', style: Desmos.Styles.DOTTED});
	calculator.setExpression({id:'field58', latex:'d_{17}=\\left(3\\cdot12+5.25\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field59', latex:'d_{18}=\\left(8\\cdot12+8\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field60', latex:'d_{19}=\\left(1\\cdot12+.75\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'field61', latex:'y=\\frac{d_3}{2}-d_{17}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\left\\{\\frac{-d_{18}}{2}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}<x<\\frac{d_{18}}{2}+d_{19}+\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\right\\}', color: '#c74440', style: Desmos.Styles.DASHED});
	calculator.setExpression({id:'field62', latex:'x=\\frac{-d_{18}}{2}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\left\\{\\frac{d_3}{2}-d_{17}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}<y<\\frac{d_3}{2}\\right\\}', color: '#c74440', style: Desmos.Styles.DASHED});
	calculator.setExpression({id:'field63', latex:'x=\\frac{d_{18}}{2}+d_{19}+\\frac{23.75}{2}\\cdot\\frac{2.54}{100}\\left\\{\\frac{d_3}{2}-d_{17}-d_{19}-\\frac{23.75}{2}\\cdot\\frac{2.54}{100}<y<\\frac{d_3}{2}\\right\\}', color: '#c74440', style: Desmos.Styles.DASHED});









   calculator.setExpression({id:'field100', latex:'d_{20}=30\\cdot\\frac{2.54}{100}'});
   calculator.setExpression({id:'field64', latex:'x\ =\ \\left(\\frac{13}{2}+13\\right)\\cdot\\frac{2.54}{100}\\left\{d_5-13\\cdot\\frac{2.54}{100}<y<d_5\\right\\}'});
   calculator.setExpression({id:'field65', latex:'x\ =\ -\\left(13\\right)\\cdot\\frac{2.54}{100}\\left\\{d_5-2\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field66', latex:'x\ =\ \\left(13\\right)\\cdot\\frac{2.54}{100}\\left\{d_5-2\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field67', latex:'x\ =\ \\left(13\\right)\\cdot\\frac{2.54}{100}\\left\\{d_5-2\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field68', latex:'x\ =\ \\left(\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\left\{d_5-3\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot2\\cdot\\frac{2.54}{100}\\right\\}'});
   calculator.setExpression({id:'field69', latex:'x\ =\ -\\left(\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\left\\{d_5-3\\cdot13\\cdot\\frac{2.54}{100}<y<d_5-13\\cdot2\\cdot\\frac{2.54}{100}\\right\\}'});
	 
   calculator.setExpression({id:'field70', latex:'\\left(y+\\left(13\\cdot2+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}-d_5\\right)^2+\\left(x\\right)^2=d_{20}^2'});
   calculator.setExpression({id:'field64', latex:'\\left(0,\ d_5-\\left(13\\cdot2+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\right)'});
   calculator.setExpression({id:'field71', latex:'\\left(\\frac{13}{2}\\cdot\\frac{2.54}{100},\ d_5-\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\right)'});
   calculator.setExpression({id:'field72', latex:'\\left(\\frac{-13}{2}\\cdot\\frac{2.54}{100},\ d_5-\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}\\right)'});
   calculator.setExpression({id:'field73', latex:'\\left(y+\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}-d_5\\right)^2+\\left(x+\\frac{13}{2}\\cdot\\frac{2.54}{100}\\right)^2=d_{20}^2'});
   calculator.setExpression({id:'field74', latex:'\\left(y+\\left(13\\cdot1+\\frac{13}{2}\\right)\\cdot\\frac{2.54}{100}-d_5\\right)^2+\\left(x-\\frac{13}{2}\\cdot\\frac{2.54}{100}\\right)^2=d_{20}^2'});


	// POSITIONS
	//calculator.setExpression({id:'position64', latex:'x=1'});
	calculator.setExpression({id:'position64', latex:'\\left(p_x,\\ p_y\\right)', color: '#c74440'});
		calculator.setExpression({id:'position65', latex:'p_x=d_1-d_2-\\left(\\frac{28}{2}+3.25\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'position66', latex:'p_y=\\left(\\frac{38.5}{2}+3.25\\right)\\cdot\\frac{2.54}{100}'});
	calculator.setExpression({id:'position67', latex:'\\left(p_{x1},\\ p_{y1}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'position68', latex:'p_{x1}=2.226'});
	calculator.setExpression({id:'position69', latex:'p_{y1}=7.31'});
	calculator.setExpression({id:'position70', latex:'\\left(p_{x2},\\ p_{y2}\\right)', color: '#388c46'});
	calculator.setExpression({id:'position71', latex:'p_{x2}=1.774'});
	calculator.setExpression({id:'position73', latex:'p_{y2}=6.25'});
	calculator.setExpression({id:'position74', latex:'\\left(p_{x3},\\ p_{y3}\\right)', color: '#fa7e19'});
	calculator.setExpression({id:'position75', latex:'p_{x3}=0.395'});
	calculator.setExpression({id:'position76', latex:'p_{y3}=5.744'});
	calculator.setExpression({id:'position77', latex:'\\left(p_{x4},\\ p_{y4}\\right)', color: '#6042a6'});
	calculator.setExpression({id:'position78', latex:'p_{x4}=2.176'});
	calculator.setExpression({id:'position79', latex:'p_{y4}=7.1'});
	calculator.setExpression({id:'position80', latex:'\\left(p_{x5},\\ p_{y5}\\right)', color: '#000000'});
	calculator.setExpression({id:'position81', latex:'p_{x5}=1.3'});
	calculator.setExpression({id:'position82', latex:'p_{y5}=5.723'});
	calculator.setExpression({id:'position83', latex:'\\left(p_{x6},\\ p_{y6}\\right)', color: '#c74440'});
	calculator.setExpression({id:'position84', latex:'p_{x6}=2.24'});
	calculator.setExpression({id:'position85', latex:'p_{y6}=7.21'});
	calculator.setExpression({id:'position86', latex:'\\left(p_{x7},\\ p_{y7}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'position87', latex:'p_{x7}=1.83'});
	calculator.setExpression({id:'position88', latex:'p_{y7}=5.56'});
	calculator.setExpression({id:'position89', latex:'\\left(p_{x8},\\ p_{y8}\\right)', color: '#000000'});
	calculator.setExpression({id:'position90', latex:'p_{x8}=0'});
	calculator.setExpression({id:'position91', latex:'p_{y8}=0'});

	







	// VELOCITIES
	calculator.setExpression({id:'velocity93', latex:'\\left(v_{xl},\\ v_{yl}\\right)', color: '#c74440'});
	calculator.setExpression({id:'velocity94', latex:'v_{xl}=2.77'});
	calculator.setExpression({id:'velocity95', latex:'v_{yl}=4.02'});
	calculator.setExpression({id:'velocity96', latex:'\\left(v_{xl1},\\ v_{yl1}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'velocity97', latex:'v_{xl1}=1.86'});
	calculator.setExpression({id:'velocity98', latex:'v_{yl1}=7.5'});
	calculator.setExpression({id:'velocity99', latex:'\\left(v_{xl2},\\ v_{yl2}\\right)', color: '#388c46'});
	calculator.setExpression({id:'velocity100', latex:'v_{xl2}=0.46'});
	calculator.setExpression({id:'velocity101', latex:'v_{yl2}=5.657'});
	calculator.setExpression({id:'velocity102', latex:'\\left(v_{xl3},\\ v_{yl3}\\right)', color: '#fa7e19'});
	calculator.setExpression({id:'velocity103', latex:'v_{xl3}=0.79'});
	calculator.setExpression({id:'velocity104', latex:'v_{yl3}=5.626'});
	calculator.setExpression({id:'velocity105', latex:'\\left(v_{xl4},\\ v_{yl4}\\right)', color: '#6042a6'});
	calculator.setExpression({id:'velocity106', latex:'v_{xl4}=2.77'});
	calculator.setExpression({id:'velocity107', latex:'v_{yl4}=9.073'});
	calculator.setExpression({id:'velocity108', latex:'\\left(v_{xl5},\\ v_{yl5}\\right)', color: '#000000'});
	calculator.setExpression({id:'velocity109', latex:'v_{xl5}=-0.204'});
	calculator.setExpression({id:'velocity110', latex:'v_{yl5}=5.475'});
	calculator.setExpression({id:'velocity111', latex:'\\left(v_{xl6},\\ v_{yl6}\\right)', color: '#c74440'});
	calculator.setExpression({id:'velocity112', latex:'v_{xl6}=1.91'});
	calculator.setExpression({id:'velocity113', latex:'v_{yl6}=8.45'});
	calculator.setExpression({id:'velocity114', latex:'\\left(v_{xl7},\\ v_{yl7}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'velocity115', latex:'v_{xl7}=1.89'});
	calculator.setExpression({id:'velocity116', latex:'v_{yl7}=4.83'});
	calculator.setExpression({id:'velocity117', latex:'\\left(v_{xl8},\\ v_{yl8}\\right)', color: '#000000'});
	calculator.setExpression({id:'velocity118', latex:'v_{xl8}=-1.1'});
	calculator.setExpression({id:'velocity119', latex:'v_{yl8}=-2.7'});


	
	// ACCELERATIONS
	calculator.setExpression({id:'acceleration120', latex:'\\left(a_{xl},\\ a_{yl}\\right)', color: '#c74440'});
	calculator.setExpression({id:'acceleration121', latex:'a_{xl}=6.49'});
	calculator.setExpression({id:'acceleration122', latex:'a_{yl}=3.72'});
	calculator.setExpression({id:'acceleration123', latex:'\\left(a_{xl1},\\ a_{yl1}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'acceleration124', latex:'a_{xl1}=1.13'});
	calculator.setExpression({id:'acceleration125', latex:'a_{yl1}=6.54'});
	calculator.setExpression({id:'acceleration126', latex:'\\left(a_{xl2},\\ a_{yl2}\\right)', color: '#388c46'});
	calculator.setExpression({id:'acceleration127', latex:'a_{xl2}=-3.5'});
	calculator.setExpression({id:'acceleration128', latex:'a_{yl2}=2.48'});
	calculator.setExpression({id:'acceleration129', latex:'\\left(a_{xl3},\\ a_{yl3}\\right)', color: '#fa7319'});
	calculator.setExpression({id:'acceleration130', latex:'a_{xl3}=12'});
	calculator.setExpression({id:'acceleration131', latex:'a_{yl3}=7.7'});
	calculator.setExpression({id:'acceleration132', latex:'\\left(a_{xl4},\\ a_{yl4}\\right)', color: '#6042a6'});
	calculator.setExpression({id:'acceleration133', latex:'a_{xl4}=2.55'});
	calculator.setExpression({id:'acceleration134', latex:'a_{yl4}=6.29'});
	calculator.setExpression({id:'acceleration135', latex:'\\left(a_{xl5},\\ a_{yl5}\\right)', color: '#000000'});
	calculator.setExpression({id:'acceleration136', latex:'a_{xl5}=0.797'});
	calculator.setExpression({id:'acceleration137', latex:'a_{yl5}=6.034'});
	calculator.setExpression({id:'acceleration138', latex:'\\left(a_{xl6},\\ a_{yl6}\\right)', color: '#c74440'});
	calculator.setExpression({id:'acceleration139', latex:'a_{xl6}=2.87'});
	calculator.setExpression({id:'acceleration140', latex:'a_{yl6}=6.91'});
	calculator.setExpression({id:'acceleration141', latex:'\\left(a_{xl7},\\ a_{yl7}\\right)', color: '#2d70b3'});
	calculator.setExpression({id:'acceleration142', latex:'a_{xl7}=2.32'});
	calculator.setExpression({id:'acceleration143', latex:'a_{yl7}=5.08'});
	calculator.setExpression({id:'acceleration144', latex:'\\left(a_{xl8},\\ a_{yl8}\\right)', color: '#000000'});
	calculator.setExpression({id:'acceleration145', latex:'a_{xl8}=-0.7'});
	calculator.setExpression({id:'acceleration146', latex:'a_{yl8}=-1.64'});




	// SPLINES
	calculator.setExpression({id:'spline147', latex:'A_x\\ =\\ \\frac{1}{2}\\left(\\left(a_{x1}-a_x\\right)-6\\left(v_x+v_{x1}\\right)+12\\left(p_{x1}-p_x\\right)\\right)'});
	calculator.setExpression({id:'spline148', latex:'B_x\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x1}\\left(2\\cdot\\left(1\\right)+3\\cdot\\left(0\\right)\\right)-a_x\\left(3\\cdot\\left(1\\right)+2\\cdot\\left(0\\right)\\right)\\right)\\ +2\\ \\left(v_x\\left(8\\cdot\\left(1\\right)+7\\cdot\\left(0\\right)\\right)+v_{x1}\\left(7\\cdot\\left(1\\right)+8\\cdot\\left(0\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x1}-\\ p_x\\right)\\left(0+1\\right)\\right)'});
	calculator.setExpression({id:'spline149', latex:'C_x\\ =\\ \\frac{1}{2}\\left(\\left(a_{x1}\\left(3\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)-a_x\\left(\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)\\right)-4\\left(v_x\\left(2\\left(0\\right)^2+10\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)+v_{x1}\\left(3\\left(0\\right)^2+10\\left(1\\right)\\cdot\\left(0\\right)+2\\left(1\\right)^2\\right)\\right)+20\\left(p_{x1}-p_x\\right)\\left(\\left(0\\right)^2+4\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline150', latex:'D_x=\\frac{a_x}{2}-10A_x\\left(0\\right)^3-6B_x\\left(0\\right)^2-3C_x\\cdot\\left(0\\right)'});
	calculator.setExpression({id:'spline151', latex:'E_x=v_x-5A_x\\left(0\\right)^4-4B_x\\left(0\\right)^3-3C_x\\left(0\\right)^2-2D_x\\left(0\\right)'});
	calculator.setExpression({id:'spline152', latex:'F_x=p_x-A_x\\left(0\\right)^5-B_x\\left(0\\right)^4-C_x\\left(0\\right)^3-D_x\\left(0\\right)^2-E_x\\left(0\\right)'});
	calculator.setExpression({id:'spline153', latex:'A_y\\ =\\ \\frac{1}{2}\\left(\\left(a_{y1}-a_y\\right)-6\\left(v_y+v_{y1}\\right)+12\\left(p_{y1}-p_y\\right)\\right)'});
	calculator.setExpression({id:'spline154', latex:'B_y\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y1}\\left(2\\cdot\\left(1\\right)+3\\cdot\\left(0\\right)\\right)-a_y\\left(3\\cdot\\left(1\\right)+2\\cdot\\left(0\\right)\\right)\\right)\\ +2\\ \\left(v_y\\left(8\\cdot\\left(1\\right)+7\\cdot\\left(0\\right)\\right)+v_{y1}\\left(7\\cdot\\left(1\\right)+8\\cdot\\left(0\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y1}-\\ p_y\\right)\\left(0+1\\right)\\right)'});
	calculator.setExpression({id:'spline155', latex:'C_y\\ =\\ \\frac{1}{2}\\left(\\left(a_{y1}\\left(3\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)-a_y\\left(\\left(0\\right)^2+6\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)\\right)-4\\left(v_y\\left(2\\left(0\\right)^2+10\\left(1\\right)\\cdot\\left(0\\right)+3\\left(1\\right)^2\\right)+v_{y1}\\left(3\\left(0\\right)^2+10\\left(1\\right)\\left(0\\right)+2\\left(1\\right)^2\\right)\\right)+20\\left(p_{y1}-p_y\\right)\\left(\\left(0\\right)^2+4\\left(1\\right)\\cdot\\left(0\\right)+\\left(1\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline156', latex:'D_y=\\frac{a_y}{2}-10A_y\\left(0\\right)^3-6B_y\\left(0\\right)^2-3C_y\\cdot\\left(0\\right)'});
	calculator.setExpression({id:'spline157', latex:'E_y=v_y-5A_y\\left(0\\right)^4-4B_y\\left(0\\right)^3-3C_y\\left(0\\right)^2-2D_y\\left(0\\right)'});
	calculator.setExpression({id:'spline159', latex:'F_y=p_y-A_y\\left(0\\right)^5-B_y\\left(0\\right)^4-C_y\\left(0\\right)^3-D_y\\left(0\\right)^2-E_y\\left(0\\right)'});
	calculator.setExpression({id:'spline160', latex:'f\\left(x\\right)\\ =\\ A_xx^5+B_xx^4+C_xx^3+D_xx^2+E_xx+F_x', hidden: true});
	calculator.setExpression({id:'spline161', latex:'g\\left(x\\right)\\ =\\ A_yx^5+B_yx^4+C_yx^3+D_yx^2+E_yx+F_y', hidden: true});
	calculator.setExpression({id:'spline162', latex:'A_{x1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x2}-a_{x1}\\right)-6\\left(v_{x1}+v_{x2}\\right)+12\\left(p_{x2}-p_{x1}\\right)\\right)'});
	calculator.setExpression({id:'spline163', latex:'B_{x1}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x2}\\left(2\\cdot\\left(2\\right)+3\\cdot\\left(1\\right)\\right)-a_{x1}\\left(3\\cdot\\left(2\\right)+2\\cdot\\left(1\\right)\\right)\\right)\\ +2\\ \\left(v_{x1}\\left(8\\cdot\\left(2\\right)+7\\cdot\\left(1\\right)\\right)+v_{x2}\\left(7\\cdot\\left(2\\right)+8\\cdot\\left(1\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x2}-\\ p_{x1}\\right)\\left(1+2\\right)\\right)'});
	calculator.setExpression({id:'spline164', latex:'C_{x1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x2}\\left(3\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)-a_{x1}\\left(\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)\\right)-4\\left(v_{x1}\\left(2\\left(1\\right)^2+10\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)+v_{x2}\\left(3\\left(1\\right)^2+10\\left(2\\right)\\cdot\\left(1\\right)+2\\left(2\\right)^2\\right)\\right)+20\\left(p_{x2}-p_{x1}\\right)\\left(\\left(1\\right)^2+4\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline165', latex:'D_{x1}=\\frac{a_{x1}}{2}-10A_{x1}\\left(1\\right)^3-6B_{x1}\\left(1\\right)^2-3C_{x1}\\cdot\\left(1\\right)'});
	calculator.setExpression({id:'spline166', latex:'E_{x1}=v_{x1}-5A_{x1}\\left(1\\right)^4-4B_{x1}\\left(1\\right)^3-3C_{x1}\\left(1\\right)^2-2D_{x1}\\left(1\\right)'});
	calculator.setExpression({id:'spline167', latex:'F_{x1}=p_{x1}-A_{x1}\\left(1\\right)^5-B_{x1}\\left(1\\right)^4-C_{x1}\\left(1\\right)^3-D_{x1}\\left(1\\right)^2-E_{x1}\\left(1\\right)'});
	calculator.setExpression({id:'spline168', latex:'A_{y1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y2}-a_{y1}\\right)-6\\left(v_{y1}+v_{y2}\\right)+12\\left(p_{y2}-p_{y1}\\right)\\right)'});
	calculator.setExpression({id:'spline169', latex:'B_{y1}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y2}\\left(2\\cdot\\left(2\\right)+3\\cdot\\left(1\\right)\\right)-a_{y1}\\left(3\\cdot\\left(2\\right)+2\\cdot\\left(1\\right)\\right)\\right)\\ +2\\ \\left(v_{y1}\\left(8\\cdot\\left(2\\right)+7\\cdot\\left(1\\right)\\right)+v_{y2}\\left(7\\cdot\\left(2\\right)+8\\cdot\\left(1\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y2}-\\ p_{y1}\\right)\\left(1+2\\right)\\right)'});
	calculator.setExpression({id:'spline170', latex:'C_{y1}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y2}\\left(3\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)-a_{y1}\\left(\\left(1\\right)^2+6\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)\\right)-4\\left(v_{y1}\\left(2\\left(1\\right)^2+10\\left(2\\right)\\cdot\\left(1\\right)+3\\left(2\\right)^2\\right)+v_{y2}\\left(3\\left(1\\right)^2+10\\left(2\\right)\\left(1\\right)+2\\left(2\\right)^2\\right)\\right)+20\\left(p_{y2}-p_{y1}\\right)\\left(\\left(1\\right)^2+4\\left(2\\right)\\cdot\\left(1\\right)+\\left(2\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline171', latex:'D_{y1}=\\frac{a_{y1}}{2}-10A_{y1}\\left(1\\right)^3-6B_{y1}\\left(1\\right)^2-3C_{y1}\\cdot\\left(1\\right)'});
	calculator.setExpression({id:'spline172', latex:'E_{y1}=v_{y1}-5A_{y1}\\left(1\\right)^4-4B_{y1}\\left(1\\right)^3-3C_{y1}\\left(1\\right)^2-2D_{y1}\\left(1\\right)'});
	calculator.setExpression({id:'spline173', latex:'F_{y1}=p_{y1}-A_{y1}\\left(1\\right)^5-B_{y1}\\left(1\\right)^4-C_{y1}\\left(1\\right)^3-D_{y1}\\left(1\\right)^2-E_{y1}\\left(1\\right)'});
	calculator.setExpression({id:'spline174', latex:'f_1\\left(x\\right)\\ =\\ A_{x1}x^5+B_{x1}x^4+C_{x1}x^3+D_{x1}x^2+E_{x1}x+F_{x1}', hidden: true});
	calculator.setExpression({id:'spline175', latex:'g_1\\left(x\\right)\\ =\\ A_{y1}x^5+B_{y1}x^4+C_{y1}x^3+D_{y1}x^2+E_{y1}x+F_{y1}', hidden: true});
	calculator.setExpression({id:'spline176', latex:'A_{x2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x3}-a_{x2}\\right)-6\\left(v_{x2}+v_{x3}\\right)+12\\left(p_{x3}-p_{x2}\\right)\\right)'});
	calculator.setExpression({id:'spline177', latex:'B_{x2}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x3}\\left(2\\cdot\\left(3\\right)+3\\cdot\\left(2\\right)\\right)-a_{x2}\\left(3\\cdot\\left(3\\right)+2\\cdot\\left(2\\right)\\right)\\right)\\ +2\\ \\left(v_{x2}\\left(8\\cdot\\left(3\\right)+7\\cdot\\left(2\\right)\\right)+v_{x3}\\left(7\\cdot\\left(3\\right)+8\\cdot\\left(2\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x3}-\\ p_{x2}\\right)\\left(2+3\\right)\\right)'});
	calculator.setExpression({id:'spline178', latex:'C_{x2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x3}\\left(3\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)-a_{x2}\\left(\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)\\right)-4\\left(v_{x2}\\left(2\\left(2\\right)^2+10\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)+v_{x3}\\left(3\\left(2\\right)^2+10\\left(3\\right)\\cdot\\left(2\\right)+2\\left(3\\right)^2\\right)\\right)+20\\left(p_{x3}-p_{x2}\\right)\\left(\\left(2\\right)^2+4\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline179', latex:'D_{x2}=\\frac{a_{x2}}{2}-10A_{x2}\\left(2\\right)^3-6B_{x2}\\left(2\\right)^2-3C_{x2}\\cdot\\left(2\\right)'});
	calculator.setExpression({id:'spline180', latex:'E_{x2}=v_{x2}-5A_{x2}\\left(2\\right)^4-4B_{x2}\\left(2\\right)^3-3C_{x2}\\left(2\\right)^2-2D_{x2}\\left(2\\right)'});
	calculator.setExpression({id:'spline181', latex:'F_{x2}=p_{x2}-A_{x2}\\left(2\\right)^5-B_{x2}\\left(2\\right)^4-C_{x2}\\left(2\\right)^3-D_{x2}\\left(2\\right)^2-E_{x2}\\left(2\\right)'});
	calculator.setExpression({id:'spline182', latex:'A_{y2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y3}-a_{y2}\\right)-6\\left(v_{y2}+v_{y3}\\right)+12\\left(p_{y3}-p_{y2}\\right)\\right)'});
	calculator.setExpression({id:'spline183', latex:'B_{y2}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y3}\\left(2\\cdot\\left(3\\right)+3\\cdot\\left(2\\right)\\right)-a_{y2}\\left(3\\cdot\\left(3\\right)+2\\cdot\\left(2\\right)\\right)\\right)\\ +2\\ \\left(v_{y2}\\left(8\\cdot\\left(3\\right)+7\\cdot\\left(2\\right)\\right)+v_{y3}\\left(7\\cdot\\left(3\\right)+8\\cdot\\left(2\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y3}-\\ p_{y2}\\right)\\left(2+3\\right)\\right)'});
	calculator.setExpression({id:'spline184', latex:'C_{y2}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y3}\\left(3\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)-a_{y2}\\left(\\left(2\\right)^2+6\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)\\right)-4\\left(v_{y2}\\left(2\\left(2\\right)^2+10\\left(3\\right)\\cdot\\left(2\\right)+3\\left(3\\right)^2\\right)+v_{y3}\\left(3\\left(2\\right)^2+10\\left(3\\right)\\left(2\\right)+2\\left(3\\right)^2\\right)\\right)+20\\left(p_{y3}-p_{y2}\\right)\\left(\\left(2\\right)^2+4\\left(3\\right)\\cdot\\left(2\\right)+\\left(3\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline185', latex:'D_{y2}=\\frac{a_{y2}}{2}-10A_{y2}\\left(2\\right)^3-6B_{y2}\\left(2\\right)^2-3C_{y2}\\cdot\\left(2\\right)'});
	calculator.setExpression({id:'spline186', latex:'E_{y2}=v_{y2}-5A_{y2}\\left(2\\right)^4-4B_{y2}\\left(2\\right)^3-3C_{y2}\\left(2\\right)^2-2D_{y2}\\left(2\\right)'});
	calculator.setExpression({id:'spline187', latex:'F_{y2}=p_{y2}-A_{y2}\\left(2\\right)^5-B_{y2}\\left(2\\right)^4-C_{y2}\\left(2\\right)^3-D_{y2}\\left(2\\right)^2-E_{y2}\\left(2\\right)'});
	calculator.setExpression({id:'spline188', latex:'f_2\\left(x\\right)\\ =\\ A_{x2}x^5+B_{x2}x^4+C_{x2}x^3+D_{x2}x^2+E_{x2}x+F_{x2}', hidden: true});
	calculator.setExpression({id:'spline189', latex:'g_2\\left(x\\right)\\ =\\ A_{y2}x^5+B_{y2}x^4+C_{y2}x^3+D_{y2}x^2+E_{y2}x+F_{y2}', hidden: true});
	calculator.setExpression({id:'spline190', latex:'A_{x3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x4}-a_{x3}\\right)-6\\left(v_{x3}+v_{x4}\\right)+12\\left(p_{x4}-p_{x3}\\right)\\right)'});
	calculator.setExpression({id:'spline191', latex:'B_{x3}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x4}\\left(2\\cdot\\left(4\\right)+3\\cdot\\left(3\\right)\\right)-a_{x3}\\left(3\\cdot\\left(4\\right)+2\\cdot\\left(3\\right)\\right)\\right)\\ +2\\ \\left(v_{x3}\\left(8\\cdot\\left(4\\right)+7\\cdot\\left(3\\right)\\right)+v_{x4}\\left(7\\cdot\\left(4\\right)+8\\cdot\\left(3\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x4}-\\ p_{x3}\\right)\\left(3+4\\right)\\right)'});
	calculator.setExpression({id:'spline192', latex:'C_{x3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x4}\\left(3\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)-a_{x3}\\left(\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)\\right)-4\\left(v_{x3}\\left(2\\left(3\\right)^2+10\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)+v_{x4}\\left(3\\left(3\\right)^2+10\\left(4\\right)\\cdot\\left(3\\right)+2\\left(4\\right)^2\\right)\\right)+20\\left(p_{x4}-p_{x3}\\right)\\left(\\left(3\\right)^2+4\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline193', latex:'D_{x3}=\\frac{a_{x3}}{2}-10A_{x3}\\left(3\\right)^3-6B_{x3}\\left(3\\right)^2-3C_{x3}\\cdot\\left(3\\right)'});
	calculator.setExpression({id:'spline194', latex:'E_{x3}=v_{x3}-5A_{x3}\\left(3\\right)^4-4B_{x3}\\left(3\\right)^3-3C_{x3}\\left(3\\right)^2-2D_{x3}\\left(3\\right)'});
	calculator.setExpression({id:'spline195', latex:'F_{x3}=p_{x3}-A_{x3}\\left(3\\right)^5-B_{x3}\\left(3\\right)^4-C_{x3}\\left(3\\right)^3-D_{x3}\\left(3\\right)^2-E_{x3}\\left(3\\right)'});
	calculator.setExpression({id:'spline196', latex:'A_{y3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y4}-a_{y3}\\right)-6\\left(v_{y3}+v_{y4}\\right)+12\\left(p_{y4}-p_{y3}\\right)\\right)'});
	calculator.setExpression({id:'spline197', latex:'B_{y3}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y4}\\left(2\\cdot\\left(4\\right)+3\\cdot\\left(3\\right)\\right)-a_{y3}\\left(3\\cdot\\left(4\\right)+2\\cdot\\left(3\\right)\\right)\\right)\\ +2\\ \\left(v_{y3}\\left(8\\cdot\\left(4\\right)+7\\cdot\\left(3\\right)\\right)+v_{y4}\\left(7\\cdot\\left(4\\right)+8\\cdot\\left(3\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y4}-\\ p_{y3}\\right)\\left(3+4\\right)\\right)'});
	calculator.setExpression({id:'spline198', latex:'C_{y3}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y4}\\left(3\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)-a_{y3}\\left(\\left(3\\right)^2+6\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)\\right)-4\\left(v_{y3}\\left(2\\left(3\\right)^2+10\\left(4\\right)\\cdot\\left(3\\right)+3\\left(4\\right)^2\\right)+v_{y4}\\left(3\\left(3\\right)^2+10\\left(4\\right)\\left(3\\right)+2\\left(4\\right)^2\\right)\\right)+20\\left(p_{y4}-p_{y3}\\right)\\left(\\left(3\\right)^2+4\\left(4\\right)\\cdot\\left(3\\right)+\\left(4\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline199', latex:'D_{y3}=\\frac{a_{y3}}{2}-10A_{y3}\\left(3\\right)^3-6B_{y3}\\left(3\\right)^2-3C_{y3}\\cdot\\left(3\\right)'});
	calculator.setExpression({id:'spline200', latex:'E_{y3}=v_{y3}-5A_{y3}\\left(3\\right)^4-4B_{y3}\\left(3\\right)^3-3C_{y3}\\left(3\\right)^2-2D_{y3}\\left(3\\right)'});
	calculator.setExpression({id:'spline201', latex:'F_{y3}=p_{y3}-A_{y3}\\left(3\\right)^5-B_{y3}\\left(3\\right)^4-C_{y3}\\left(3\\right)^3-D_{y3}\\left(3\\right)^2-E_{y3}\\left(3\\right)'});
	calculator.setExpression({id:'spline202', latex:'f_3\\left(x\\right)\\ =\\ A_{x3}x^5+B_{x3}x^4+C_{x3}x^3+D_{x3}x^2+E_{x3}x+F_{x3}', hidden: true});
	calculator.setExpression({id:'spline203', latex:'g_3\\left(x\\right)\\ =\\ A_{y3}x^5+B_{y3}x^4+C_{y3}x^3+D_{y3}x^2+E_{y3}x+F_{y3}', hidden: true});
	calculator.setExpression({id:'spline204', latex:'A_{x4}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x5}-a_{x4}\\right)-6\\left(v_{x4}+v_{x5}\\right)+12\\left(p_{x5}-p_{x4}\\right)\\right)'});
	calculator.setExpression({id:'spline205', latex:'B_{x4}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x5}\\left(2\\cdot\\left(5\\right)+3\\cdot\\left(4\\right)\\right)-a_{x4}\\left(3\\cdot\\left(5\\right)+2\\cdot\\left(4\\right)\\right)\\right)\\ +2\\ \\left(v_{x4}\\left(8\\cdot\\left(5\\right)+7\\cdot\\left(4\\right)\\right)+v_{x5}\\left(7\\cdot\\left(5\\right)+8\\cdot\\left(4\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x5}-\\ p_{x4}\\right)\\left(5+4\\right)\\right)'});
	calculator.setExpression({id:'spline206', latex:'C_{x4}=\\ \\frac{1}{2}\\left(\\left(a_{x5}\\left(3\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)-a_{x4}\\left(\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)\\right)-4\\left(v_{x4}\\left(2\\left(4\\right)^2+10\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)+v_{x5}\\left(3\\left(4\\right)^2+10\\left(5\\right)\\cdot\\left(4\\right)+2\\left(5\\right)^2\\right)\\right)+20\\left(p_{x5}-p_{x4}\\right)\\left(\\left(4\\right)^2+4\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline207', latex:'D_{x4}=\\frac{a_{x4}}{2}-10A_{x4}\\left(4\\right)^3-6B_{x4}\\left(4\\right)^2-3C_{x4}\\cdot\\left(4\\right)'});
	calculator.setExpression({id:'spline208', latex:'E_{x4}=v_{x4}-5A_{x4}\\left(4\\right)^4-4B_{x4}\\left(4\\right)^3-3C_{x4}\\left(4\\right)^2-2D_{x4}\\left(4\\right)'});
	calculator.setExpression({id:'spline209', latex:'F_{x4}=p_{x4}-A_{x4}\\left(4\\right)^5-B_{x4}\\left(4\\right)^4-C_{x4}\\left(4\\right)^3-D_{x4}\\left(4\\right)^2-E_{x4}\\left(4\\right)'});
	calculator.setExpression({id:'spline210', latex:'A_{y4}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y5}-a_{y4}\\right)-6\\left(v_{y4}+v_{y5}\\right)+12\\left(p_{y5}-p_{y4}\\right)\\right)'});
	calculator.setExpression({id:'spline211', latex:'B_{y4}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y5}\\left(2\\cdot\\left(5\\right)+3\\cdot\\left(4\\right)\\right)-a_{y4}\\left(3\\cdot\\left(5\\right)+2\\cdot\\left(4\\right)\\right)\\right)\\ +2\\ \\left(v_{y4}\\left(8\\cdot\\left(5\\right)+7\\cdot\\left(4\\right)\\right)+v_{y5}\\left(7\\cdot\\left(5\\right)+8\\cdot\\left(4\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y5}-\\ p_{y4}\\right)\\left(5+4\\right)\\right)'});
	calculator.setExpression({id:'spline212', latex:'C_{y4}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y5}\\left(3\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)-a_{y4}\\left(\\left(4\\right)^2+6\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)\\right)-4\\left(v_{y4}\\left(2\\left(4\\right)^2+10\\left(5\\right)\\cdot\\left(4\\right)+3\\left(5\\right)^2\\right)+v_{y5}\\left(3\\left(4\\right)^2+10\\left(5\\right)\\left(4\\right)+2\\left(5\\right)^2\\right)\\right)+20\\left(p_{y5}-p_{y4}\\right)\\left(\\left(4\\right)^2+4\\left(5\\right)\\cdot\\left(4\\right)+\\left(5\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline213', latex:'D_{y4}=\\frac{a_{y4}}{2}-10A_{y4}\\left(4\\right)^3-6B_{y4}\\left(4\\right)^2-3C_{y4}\\cdot\\left(4\\right)'});
	calculator.setExpression({id:'spline214', latex:'E_{y4}=v_{y4}-5A_{y4}\\left(4\\right)^4-4B_{y4}\\left(4\\right)^3-3C_{y4}\\left(4\\right)^2-2D_{y4}\\left(4\\right)'});
	calculator.setExpression({id:'spline215', latex:'F_{y4}=p_{y4}-A_{y4}\\left(4\\right)^5-B_{y4}\\left(4\\right)^4-C_{y4}\\left(4\\right)^3-D_{y4}\\left(4\\right)^2-E_{y4}\\left(4\\right)'});
	calculator.setExpression({id:'spline216', latex:'f_4\\left(x\\right)\\ =\\ A_{x4}x^5+B_{x4}x^4+C_{x4}x^3+D_{x4}x^2+E_{x4}x+F_{x4}', hidden: true});
	calculator.setExpression({id:'spline217', latex:'g_4\\left(x\\right)\\ =\\ A_{y4}x^5+B_{y4}x^4+C_{y4}x^3+D_{y4}x^2+E_{y4}x+F_{y4}', hidden: true});
	calculator.setExpression({id:'spline218', latex:'A_{x5}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x6}-a_{x5}\\right)-6\\left(v_{x5}+v_{x6}\\right)+12\\left(p_{x6}-p_{x5}\\right)\\right)'});
	calculator.setExpression({id:'spline219', latex:'B_{x5}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x6}\\left(2\\cdot\\left(6\\right)+3\\cdot\\left(5\\right)\\right)-a_{x5}\\left(3\\cdot\\left(6\\right)+2\\cdot\\left(5\\right)\\right)\\right)\\ +2\\ \\left(v_{x5}\\left(8\\cdot\\left(6\\right)+7\\cdot\\left(5\\right)\\right)+v_{x6}\\left(7\\cdot\\left(6\\right)+8\\cdot\\left(5\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x6}-\\ p_{x5}\\right)\\left(6+5\\right)\\right)'});
	calculator.setExpression({id:'spline220', latex:'C_{x5}=\\ \\frac{1}{2}\\left(\\left(a_{x6}\\left(3\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)-a_{x5}\\left(\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)\\right)-4\\left(v_{x5}\\left(2\\left(5\\right)^2+10\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)+v_{x6}\\left(3\\left(5\\right)^2+10\\left(6\\right)\\cdot\\left(5\\right)+2\\left(6\\right)^2\\right)\\right)+20\\left(p_{x6}-p_{x5}\\right)\\left(\\left(5\\right)^2+4\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline221', latex:'D_{x5}=\\frac{a_{x5}}{2}-10A_{x5}\\left(5\\right)^3-6B_{x5}\\left(5\\right)^2-3C_{x5}\\cdot\\left(5\\right)'});
	calculator.setExpression({id:'spline222', latex:'E_{x5}=v_{x5}-5A_{x5}\\left(5\\right)^4-4B_{x5}\\left(5\\right)^3-3C_{x5}\\left(5\\right)^2-2D_{x5}\\left(5\\right)'});
	calculator.setExpression({id:'spline223', latex:'F_{x5}=p_{x5}-A_{x5}\\left(5\\right)^5-B_{x5}\\left(5\\right)^4-C_{x5}\\left(5\\right)^3-D_{x5}\\left(5\\right)^2-E_{x5}\\left(5\\right)'});
	calculator.setExpression({id:'spline224', latex:'A_{y5}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y6}-a_{y5}\\right)-6\\left(v_{y5}+v_{y6}\\right)+12\\left(p_{y6}-p_{y5}\\right)\\right)'});
	calculator.setExpression({id:'spline225', latex:'B_{y5}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y6}\\left(2\\cdot\\left(6\\right)+3\\cdot\\left(5\\right)\\right)-a_{y5}\\left(3\\cdot\\left(6\\right)+2\\cdot\\left(5\\right)\\right)\\right)\\ +2\\ \\left(v_{y5}\\left(8\\cdot\\left(6\\right)+7\\cdot\\left(5\\right)\\right)+v_{y6}\\left(7\\cdot\\left(6\\right)+8\\cdot\\left(5\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y6}-\\ p_{y5}\\right)\\left(6+5\\right)\\right)'});
	calculator.setExpression({id:'spline226', latex:'C_{y5}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y6}\\left(3\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)-a_{y5}\\left(\\left(5\\right)^2+6\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)\\right)-4\\left(v_{y5}\\left(2\\left(5\\right)^2+10\\left(6\\right)\\cdot\\left(5\\right)+3\\left(6\\right)^2\\right)+v_{y6}\\left(3\\left(5\\right)^2+10\\left(6\\right)\\left(5\\right)+2\\left(6\\right)^2\\right)\\right)+20\\left(p_{y6}-p_{y5}\\right)\\left(\\left(5\\right)^2+4\\left(6\\right)\\cdot\\left(5\\right)+\\left(6\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline227', latex:'D_{y5}=\\frac{a_{y5}}{2}-10A_{y5}\\left(5\\right)^3-6B_{y5}\\left(5\\right)^2-3C_{y5}\\cdot\\left(5\\right)'});
	calculator.setExpression({id:'spline228', latex:'E_{y5}=v_{y5}-5A_{y5}\\left(5\\right)^4-4B_{y5}\\left(5\\right)^3-3C_{y5}\\left(5\\right)^2-2D_{y5}\\left(5\\right)'});
	calculator.setExpression({id:'spline229', latex:'F_{y5}=p_{y5}-A_{y5}\\left(5\\right)^5-B_{y5}\\left(5\\right)^4-C_{y5}\\left(5\\right)^3-D_{y5}\\left(5\\right)^2-E_{y5}\\left(5\\right)'});
	calculator.setExpression({id:'spline230', latex:'f_5\\left(x\\right)\\ =\\ A_{x5}x^5+B_{x5}x^4+C_{x5}x^3+D_{x5}x^2+E_{x5}x+F_{x5}', hidden: true});
	calculator.setExpression({id:'spline231', latex:'g_5\\left(x\\right)\\ =\\ A_{y5}x^5+B_{y5}x^4+C_{y5}x^3+D_{y5}x^2+E_{y5}x+F_{y5}', hidden: true});
	calculator.setExpression({id:'spline232', latex:'A_{x6}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x7}-a_{x6}\\right)-6\\left(v_{x6}+v_{x7}\\right)+12\\left(p_{x7}-p_{x6}\\right)\\right)'});
	calculator.setExpression({id:'spline233', latex:'B_{x6}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x7}\\left(2\\cdot\\left(7\\right)+3\\cdot\\left(6\\right)\\right)-a_{x6}\\left(3\\cdot\\left(7\\right)+2\\cdot\\left(6\\right)\\right)\\right)\\ +2\\ \\left(v_{x6}\\left(8\\cdot\\left(7\\right)+7\\cdot\\left(6\\right)\\right)+v_{x7}\\left(7\\cdot\\left(7\\right)+8\\cdot\\left(6\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x7}-\\ p_{x6}\\right)\\left(7+6\\right)\\right)'});
	calculator.setExpression({id:'spline234', latex:'C_{x6}=\\ \\frac{1}{2}\\left(\\left(a_{x7}\\left(3\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)-a_{x6}\\left(\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)\\right)-4\\left(v_{x6}\\left(2\\left(6\\right)^2+10\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)+v_{x7}\\left(3\\left(6\\right)^2+10\\left(7\\right)\\cdot\\left(6\\right)+2\\left(7\\right)^2\\right)\\right)+20\\left(p_{x7}-p_{x6}\\right)\\left(\\left(6\\right)^2+4\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline235', latex:'D_{x6}=\\frac{a_{x6}}{2}-10A_{x6}\\left(6\\right)^3-6B_{x6}\\left(6\\right)^2-3C_{x6}\\cdot\\left(6\\right)'});
	calculator.setExpression({id:'spline236', latex:'E_{x6}=v_{x6}-5A_{x6}\\left(6\\right)^4-4B_{x6}\\left(6\\right)^3-3C_{x6}\\left(6\\right)^2-2D_{x6}\\left(6\\right)'});
	calculator.setExpression({id:'spline237', latex:'F_{x6}=p_{x6}-A_{x6}\\left(6\\right)^5-B_{x6}\\left(6\\right)^4-C_{x6}\\left(6\\right)^3-D_{x6}\\left(6\\right)^2-E_{x6}\\left(6\\right)'});
	calculator.setExpression({id:'spline238', latex:'A_{y6}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y7}-a_{y6}\\right)-6\\left(v_{y6}+v_{y7}\\right)+12\\left(p_{y7}-p_{y6}\\right)\\right)'});
	calculator.setExpression({id:'spline239', latex:'B_{y6}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y7}\\left(2\\cdot\\left(7\\right)+3\\cdot\\left(6\\right)\\right)-a_{y6}\\left(3\\cdot\\left(7\\right)+2\\cdot\\left(6\\right)\\right)\\right)\\ +2\\ \\left(v_{y6}\\left(8\\cdot\\left(7\\right)+7\\cdot\\left(6\\right)\\right)+v_{y7}\\left(7\\cdot\\left(7\\right)+8\\cdot\\left(6\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y7}-\\ p_{y6}\\right)\\left(7+6\\right)\\right)'});
	calculator.setExpression({id:'spline240', latex:'C_{y6}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y7}\\left(3\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)-a_{y6}\\left(\\left(6\\right)^2+6\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)\\right)-4\\left(v_{y6}\\left(2\\left(6\\right)^2+10\\left(7\\right)\\cdot\\left(6\\right)+3\\left(7\\right)^2\\right)+v_{y7}\\left(3\\left(6\\right)^2+10\\left(7\\right)\\left(6\\right)+2\\left(7\\right)^2\\right)\\right)+20\\left(p_{y7}-p_{y6}\\right)\\left(\\left(6\\right)^2+4\\left(7\\right)\\cdot\\left(6\\right)+\\left(7\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline241', latex:'D_{y6}=\\frac{a_{y6}}{2}-10A_{y6}\\left(6\\right)^3-6B_{y6}\\left(6\\right)^2-3C_{y6}\\cdot\\left(6\\right)'});
	calculator.setExpression({id:'spline242', latex:'E_{y6}=v_{y6}-5A_{y6}\\left(6\\right)^4-4B_{y6}\\left(6\\right)^3-3C_{y6}\\left(6\\right)^2-2D_{y6}\\left(6\\right)'});
	calculator.setExpression({id:'spline243', latex:'F_{y6}=p_{y6}-A_{y6}\\left(6\\right)^5-B_{y6}\\left(6\\right)^4-C_{y6}\\left(6\\right)^3-D_{y6}\\left(6\\right)^2-E_{y6}\\left(6\\right)'});
	calculator.setExpression({id:'spline244', latex:'f_6\\left(x\\right)\\ =\\ A_{x6}x^5+B_{x6}x^4+C_{x6}x^3+D_{x6}x^2+E_{x6}x+F_{x6}', hidden: true});
	calculator.setExpression({id:'spline245', latex:'g_6\\left(x\\right)\\ =\\ A_{y6}x^5+B_{y6}x^4+C_{y6}x^3+D_{y6}x^2+E_{y6}x+F_{y6}', hidden: true});
	calculator.setExpression({id:'spline246', latex:'A_{x7}\\ =\\ \\frac{1}{2}\\left(\\left(a_{x8}-a_{x7}\\right)-6\\left(v_{x7}+v_{x8}\\right)+12\\left(p_{x8}-p_{x7}\\right)\\right)'});
	calculator.setExpression({id:'spline247', latex:'B_{x7}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{x8}\\left(2\\cdot\\left(8\\right)+3\\cdot\\left(7\\right)\\right)-a_{x7}\\left(3\\cdot\\left(8\\right)+2\\cdot\\left(7\\right)\\right)\\right)\\ +2\\ \\left(v_{x7}\\left(8\\cdot\\left(8\\right)+7\\cdot\\left(7\\right)\\right)+v_{x8}\\left(7\\cdot\\left(8\\right)+8\\cdot\\left(7\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{x8}-\\ p_{x7}\\right)\\left(8+7\\right)\\right)'});
	calculator.setExpression({id:'spline248', latex:'C_{x7}=\\ \\frac{1}{2}\\left(\\left(a_{x8}\\left(3\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)-a_{x7}\\left(\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)\\right)-4\\left(v_{x7}\\left(2\\left(7\\right)^2+10\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)+v_{x8}\\left(3\\left(7\\right)^2+10\\left(8\\right)\\cdot\\left(7\\right)+2\\left(8\\right)^2\\right)\\right)+20\\left(p_{x8}-p_{x7}\\right)\\left(\\left(7\\right)^2+4\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline249', latex:'D_{x7}=\\frac{a_{x7}}{2}-10A_{x7}\\left(7\\right)^3-6B_{x7}\\left(7\\right)^2-3C_{x7}\\cdot\\left(7\\right)'});
	calculator.setExpression({id:'spline250', latex:'E_{x7}=v_{x7}-5A_{x7}\\left(7\\right)^4-4B_{x7}\\left(7\\right)^3-3C_{x7}\\left(7\\right)^2-2D_{x7}\\left(7\\right)'});
	calculator.setExpression({id:'spline251', latex:'F_{x7}=p_{x7}-A_{x7}\\left(7\\right)^5-B_{x7}\\left(7\\right)^4-C_{x7}\\left(7\\right)^3-D_{x7}\\left(7\\right)^2-E_{x7}\\left(7\\right)'});
	calculator.setExpression({id:'spline252', latex:'A_{y7}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y8}-a_{y7}\\right)-6\\left(v_{y7}+v_{y8}\\right)+12\\left(p_{y8}-p_{y7}\\right)\\right)'});
	calculator.setExpression({id:'spline253', latex:'12\\left(p_{y8}-p_{y7}\\right)'});
	calculator.setExpression({id:'spline254', latex:'B_{y7}\\ =\\ \\frac{1}{2}\\left(-\\left(a_{y8}\\left(2\\cdot\\left(8\\right)+3\\cdot\\left(7\\right)\\right)-a_{y7}\\left(3\\cdot\\left(8\\right)+2\\cdot\\left(7\\right)\\right)\\right)\\ +2\\ \\left(v_{y7}\\left(8\\cdot\\left(8\\right)+7\\cdot\\left(7\\right)\\right)+v_{y8}\\left(7\\cdot\\left(8\\right)+8\\cdot\\left(7\\right)\\right)\\right)\\ -\\ 30\\ \\left(p_{y8}-\\ p_{y7}\\right)\\left(8+7\\right)\\right)'});
	calculator.setExpression({id:'spline255', latex:'C_{y7}\\ =\\ \\frac{1}{2}\\left(\\left(a_{y8}\\left(3\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)-a_{y7}\\left(\\left(7\\right)^2+6\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)\\right)-4\\left(v_{y7}\\left(2\\left(7\\right)^2+10\\left(8\\right)\\cdot\\left(7\\right)+3\\left(8\\right)^2\\right)+v_{y8}\\left(3\\left(7\\right)^2+10\\left(8\\right)\\left(7\\right)+2\\left(8\\right)^2\\right)\\right)+20\\left(p_{y8}-p_{y7}\\right)\\left(\\left(7\\right)^2+4\\left(8\\right)\\cdot\\left(7\\right)+\\left(8\\right)^2\\right)\\right)'});
	calculator.setExpression({id:'spline256', latex:'D_{y7}=\\frac{a_{y7}}{2}-10A_{y7}\\left(7\\right)^3-6B_{y7}\\left(7\\right)^2-3C_{y7}\\cdot\\left(7\\right)'});
	calculator.setExpression({id:'spline257', latex:'E_{y7}=v_{y7}-5A_{y7}\\left(7\\right)^4-4B_{y7}\\left(7\\right)^3-3C_{y7}\\left(7\\right)^2-2D_{y7}\\left(7\\right)'});
	calculator.setExpression({id:'spline258', latex:'F_{y7}=p_{y7}-A_{y7}\\left(7\\right)^5-B_{y7}\\left(7\\right)^4-C_{y7}\\left(7\\right)^3-D_{y7}\\left(7\\right)^2-E_{y7}\\left(7\\right)'});
	calculator.setExpression({id:'spline259', latex:'f_7\\left(x\\right)\\ =\\ A_{x7}x^5+B_{x7}x^4+C_{x7}x^3+D_{x7}x^2+E_{x7}x+F_{x7}', hidden: true});
	calculator.setExpression({id:'spline260', latex:'g_7\\left(x\\right)\\ =\\ A_{y7}x^5+B_{y7}x^4+C_{y7}x^3+D_{y7}x^2+E_{y7}x+F_{y7}', hidden: true});


	






	// OUT
	calculator.setExpression({id:'out261', latex:'v_x\\ =\\left(\\ v_{xl}-p_x\\right)'});
	calculator.setExpression({id:'out262', latex:'v_y\\ =\\ \\left(\\ v_{yl}-p_y\\right)'});
	calculator.setExpression({id:'out263', latex:'v_{x1}\\ =\\left(\\ v_{xl1}-p_{x1}\\right)'});
	calculator.setExpression({id:'out264', latex:'v_{y1}\\ =\\ \\left(v_{yl1}-p_{y1}\\right)'});
	calculator.setExpression({id:'out265', latex:'v_{x2}\\ =\\left(\\ v_{xl2}-p_{x2}\\right)'});
	calculator.setExpression({id:'out266', latex:'v_{y2}\\ =\\ v_{yl2}-p_{y2}'});
	calculator.setExpression({id:'out267', latex:'v_{x3}\\ =\\left(\\ v_{xl3}-p_{x3}\\right)'});
	calculator.setExpression({id:'out268', latex:'v_{y3}=\\ \\left(v_{yl3}-p_{y3}\\right)'});
	calculator.setExpression({id:'out269', latex:'v_{x4}\\ =\\left(\\ v_{xl4}-p_{x4}\\right)'});
	calculator.setExpression({id:'out270', latex:'v_{y4}=\\ \\left(v_{yl4}-p_{y4}\\right)'});
	calculator.setExpression({id:'out271', latex:'v_{x5}\\ =\\left(\\ v_{xl5}-p_{x5}\\right)'});
	calculator.setExpression({id:'out272', latex:'v_{y5}=\\ \\left(v_{yl5}-p_{y5}\\right)'});
	calculator.setExpression({id:'out273', latex:'v_{x6}\\ =\\left(\\ v_{xl6}-p_{x6}\\right)'});
	calculator.setExpression({id:'out274', latex:'v_{y6}=\\ \\left(v_{yl6}-p_{y6}\\right)'});
	calculator.setExpression({id:'out275', latex:'v_{x7}\\ =\\left(\\ v_{xl7}-p_{x7}\\right)'});
	calculator.setExpression({id:'out276', latex:'v_{y7}=\\ \\left(v_{yl7}-p_{y7}\\right)'});
	calculator.setExpression({id:'out277', latex:'v_{x8}\\ =\\left(\\ v_{xl8}-p_{x8}\\right)'});
	calculator.setExpression({id:'out278', latex:'v_{y8}=\\ \\left(v_{yl8}-p_{y8}\\right)'});
	calculator.setExpression({id:'out279', latex:'a_x\\ =a_{xl}-p_x-v_x'});
	calculator.setExpression({id:'out280', latex:'a_y\\ =\\ a_{yl}-p_y-v_y'});
	calculator.setExpression({id:'out281', latex:'\\left(a_xt^2+\\left(v_x-2\\cdot\\left(0\\right)\\cdot a_x\\right)t+\\left(p_x-\\left(0\\right)\\cdot\\left(v_x-2\\cdot\\left(0\\right)\\cdot a_x\\right)-\\left(0\\right)^2\\cdot a_x\\right),\\ a_yt^2+\\left(v_y-2\\cdot\\left(0\\right)\\cdot a_y\\right)t+\\left(p_y-\\left(0\\right)\\cdot\\left(v_y-2\\cdot\\left(0\\right)\\cdot a_y\\right)-\\left(0\\right)^2\\cdot a_y\\right)\\right)', hidden: true, domain:{min: -1, max: 1}});
	calculator.setExpression({id:'out282', latex:'a_{x1}\\ =a_{xl1}-p_{x1}-v_{x1}'});
	calculator.setExpression({id:'out283', latex:'a_{y1}\\ =\\ a_{yl1}-p_{y1}-v_{y1}'});
	calculator.setExpression({id:'out284', latex:'\\left(a_{x1}t^2+\\left(v_{x1}-2\\cdot\\left(1\\right)\\cdot a_{x1}\\right)t+\\left(p_{x1}-\\left(1\\right)\\cdot\\left(v_{x1}-2\\cdot\\left(1\\right)\\cdot a_{x1}\\right)-\\left(1\\right)^2\\cdot a_{x1}\\right),\\ a_{y1}t^2+\\left(v_{y1}-2\\cdot\\left(1\\right)\\cdot a_{y1}\\right)t+\\left(p_{y1}-\\left(1\\right)\\cdot\\left(v_{y1}-2\\cdot\\left(1\\right)\\cdot a_{y1}\\right)-\\left(1\\right)^2\\cdot a_{y1}\\right)\\right)', hidden: true, domain:{min: 0, max: 2}});
	calculator.setExpression({id:'out285', latex:'a_{x2}\\ =a_{xl2}-p_{x2}-v_{x2}'});
	calculator.setExpression({id:'out286', latex:'a_{y2}\\ =\\ a_{yl2}-p_{y2}-v_{y2}'});
	calculator.setExpression({id:'out287', latex:'\\left(a_{x2}t^2+\\left(v_{x2}-2\\cdot\\left(2\\right)\\cdot a_{x2}\\right)t+\\left(p_{x2}-\\left(2\\right)\\cdot\\left(v_{x2}-2\\cdot\\left(2\\right)\\cdot a_{x2}\\right)-\\left(2\\right)^2\\cdot a_{x2}\\right),\\ a_{y2}t^2+\\left(v_{y2}-2\\cdot\\left(2\\right)\\cdot a_{y2}\\right)t+\\left(p_{y2}-\\left(2\\right)\\cdot\\left(v_{y2}-2\\cdot\\left(2\\right)\\cdot a_{y2}\\right)-\\left(2\\right)^2\\cdot a_{y2}\\right)\\right)', hidden: true, domain:{min: 1, max: 3}});
	calculator.setExpression({id:'out288', latex:'a_{x3}\\ =a_{xl3}-p_{x3}-v_{x3}'});
	calculator.setExpression({id:'out289', latex:'a_{y3}\\ =\\ a_{yl3}-p_{y3}-v_{y3}'});
	calculator.setExpression({id:'out290', latex:'\\left(a_{x3}t^2+\\left(v_{x3}-2\\cdot\\left(3\\right)\\cdot a_{x3}\\right)t+\\left(p_{x3}-\\left(3\\right)\\cdot\\left(v_{x3}-2\\cdot\\left(3\\right)\\cdot a_{x3}\\right)-\\left(3\\right)^2\\cdot a_{x3}\\right),\\ a_{y3}t^2+\\left(v_{y3}-2\\cdot\\left(3\\right)\\cdot a_{y3}\\right)t+\\left(p_{y3}-\\left(3\\right)\\cdot\\left(v_{y3}-2\\cdot\\left(3\\right)\\cdot a_{y3}\\right)-\\left(3\\right)^2\\cdot a_{y3}\\right)\\right)', hidden: true, domain:{min: 2, max: 4}});
	calculator.setExpression({id:'out291', latex:'a_{x4}\\ =a_{xl4}-p_{x4}-v_{x4}'});
	calculator.setExpression({id:'out292', latex:'a_{y4}\\ =\\ a_{yl4}-p_{y4}-v_{y4}'});
	calculator.setExpression({id:'out293', latex:'\\left(a_{x4}t^2+\\left(v_{x4}-2\\cdot\\left(4\\right)\\cdot a_{x4}\\right)t+\\left(p_{x4}-\\left(4\\right)\\cdot\\left(v_{x4}-2\\cdot\\left(4\\right)\\cdot a_{x4}\\right)-\\left(4\\right)^2\\cdot a_{x4}\\right),\\ a_{y4}t^2+\\left(v_{y4}-2\\cdot\\left(4\\right)\\cdot a_{y4}\\right)t+\\left(p_{y4}-\\left(4\\right)\\cdot\\left(v_{y4}-2\\cdot\\left(4\\right)\\cdot a_{y4}\\right)-\\left(4\\right)^2\\cdot a_{y4}\\right)\\right)', hidden: true, domain:{min: 3, max: 5}});
	calculator.setExpression({id:'out294', latex:'a_{x5}\\ =a_{xl5}-p_{x5}-v_{x5}'});
	calculator.setExpression({id:'out295', latex:'a_{y5}\\ =\\ a_{yl5}-p_{y5}-v_{y5}'});
	calculator.setExpression({id:'out296', latex:'\\left(a_{x5}t^2+\\left(v_{x5}-2\\cdot\\left(5\\right)\\cdot a_{x5}\\right)t+\\left(p_{x5}-\\left(5\\right)\\cdot\\left(v_{x5}-2\\cdot\\left(5\\right)\\cdot a_{x5}\\right)-\\left(5\\right)^2\\cdot a_{x5}\\right),\\ a_{y5}t^2+\\left(v_{y5}-2\\cdot\\left(5\\right)\\cdot a_{y5}\\right)t+\\left(p_{y5}-\\left(5\\right)\\cdot\\left(v_{y5}-2\\cdot\\left(5\\right)\\cdot a_{y5}\\right)-\\left(5\\right)^2\\cdot a_{y5}\\right)\\right)', hidden: true, domain:{min: 4, max: 6}});
	calculator.setExpression({id:'out297', latex:'a_{x6}\\ =a_{xl6}-p_{x6}-v_{x6}'});
	calculator.setExpression({id:'out298', latex:'a_{y6}\\ =\\ a_{yl6}-p_{y6}-v_{y6}'});
	calculator.setExpression({id:'out299', latex:'\\left(a_{x6}t^2+\\left(v_{x6}-2\\cdot\\left(6\\right)\\cdot a_{x6}\\right)t+\\left(p_{x6}-\\left(6\\right)\\cdot\\left(v_{x6}-2\\cdot\\left(6\\right)\\cdot a_{x6}\\right)-\\left(6\\right)^2\\cdot a_{x6}\\right),\\ a_{y6}t^2+\\left(v_{y6}-2\\cdot\\left(6\\right)\\cdot a_{y6}\\right)t+\\left(p_{y6}-\\left(6\\right)\\cdot\\left(v_{y6}-2\\cdot\\left(6\\right)\\cdot a_{y6}\\right)-\\left(6\\right)^2\\cdot a_{y6}\\right)\\right)', hidden: true, domain:{min: 5, max: 7}});
	calculator.setExpression({id:'out300', latex:'a_{x7}\\ =a_{xl7}-p_{x7}-v_{x7}'});
	calculator.setExpression({id:'out301', latex:'a_{y7}\\ =\\ a_{yl7}-p_{y7}-v_{y7}'});
	calculator.setExpression({id:'out302', latex:'\\left(a_{x7}t^2+\\left(v_{x7}-2\\cdot\\left(7\\right)\\cdot a_{x7}\\right)t+\\left(p_{x7}-\\left(7\\right)\\cdot\\left(v_{x7}-2\\cdot\\left(7\\right)\\cdot a_{x7}\\right)-\\left(7\\right)^2\\cdot a_{x7}\\right),\\ a_{y7}t^2+\\left(v_{y7}-2\\cdot\\left(7\\right)\\cdot a_{y7}\\right)t+\\left(p_{y7}-\\left(7\\right)\\cdot\\left(v_{y7}-2\\cdot\\left(7\\right)\\cdot a_{y7}\\right)-\\left(7\\right)^2\\cdot a_{y7}\\right)\\right)', hidden: true, domain:{min: 6, max: 8}});
	calculator.setExpression({id:'out303', latex:'a_{x8}\\ =a_{xl8}-p_{x8}-v_{x8}'});
	calculator.setExpression({id:'out304', latex:'a_{y8}\\ =\\ a_{yl8}-p_{y8}-v_{y8}'});
	calculator.setExpression({id:'out305', latex:'\\left(a_{x8}t^2+\\left(v_{x8}-2\\cdot\\left(8\\right)\\cdot a_{x8}\\right)t+\\left(p_{x8}-\\left(8\\right)\\cdot\\left(v_{x8}-2\\cdot\\left(8\\right)\\cdot a_{x8}\\right)-\\left(8\\right)^2\\cdot a_{x8}\\right),\\ a_{y8}t^2+\\left(v_{y8}-2\\cdot\\left(8\\right)\\cdot a_{y8}\\right)t+\\left(p_{y8}-\\left(8\\right)\\cdot\\left(v_{y8}-2\\cdot\\left(8\\right)\\cdot a_{y8}\\right)-\\left(8\\right)^2\\cdot a_{y8}\\right)\\right)', hidden: true, domain:{min: 7, max: 9}});
	
	// FUNCTIONS
	
	calculator.setExpression({id:'func306', latex:'\\left(f\\left(t\\right),\\ g\\left(t\\right)\\right)', color: '#c74440', domain:{min: 0, max: 1}});
	calculator.setExpression({id:'func307', latex:'\\left(f_1\\left(t\\right),\\ g_1\\left(t\\right)\\right)', domain:{min: 1, max: 2}});
	calculator.setExpression({id:'func308', latex:'\\left(f_2\\left(t\\right),\\ g_2\\left(t\\right)\\right)', domain:{min: 2, max: 3}});
	calculator.setExpression({id:'func309', latex:'\\left(f_3\\left(t\\right),\\ g_3\\left(t\\right)\\right)', domain:{min: 3, max: 4}});
	calculator.setExpression({id:'func310', latex:'\\left(f_4\\left(t\\right),\\ g_4\\left(t\\right)\\right)', domain:{min: 4, max: 5}});
	calculator.setExpression({id:'func311', latex:'\\left(f_5\\left(t\\right),\\ g_5\\left(t\\right)\\right)', domain:{min: 5, max: 6}});
	calculator.setExpression({id:'func312', latex:'\\left(f_6\\left(t\\right),\\ g_6\\left(t\\right)\\right)', color: '#c74440', domain:{min: 6, max: 7}});
	calculator.setExpression({id:'func315', latex:'\\left(f_7\\left(t\\right),\\ g_7\\left(t\\right)\\right)', color: '#000000', domain:{min: 7, max: 8}});
}
/*function () {
var textFile = null,
  makeTextFile = function (text) {
    var data = new Blob([text], {type: 'text/plain'});

    // If we are replacing a previously generated file we need to
    // manually revoke the object URL to avoid memory leaks.
    if (textFile !== null) {
      window.URL.revokeObjectURL(textFile);
    }

    textFile = window.URL.createObjectURL(data);

    return textFile;
  };
var create = document.getElementById('create'),
    textbox = document.getElementById('textbox');

  create.addEventListener('click', function () {
    var link = document.getElementById('downloadlink');
    link.href = makeTextFile(textbox.value);
    link.style.display = 'block';
  }, false);
})();*/



/*function download_coefs()
{
    


var textFile = null,



  var create = document.getElementById('create'),
    textbox = document.getElementById('textbox');

  create.addEventListener('click', function () {
    var link = document.getElementById('downloadlink');
    link.href = makeTextFile(textbox.value);
    link.style.display = 'block';


}*/
