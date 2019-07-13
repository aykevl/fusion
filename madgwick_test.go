package fusion

import (
	"testing"
	"time"

	"github.com/go-gl/mathgl/mgl32"
)

var madgwickTestFixture = []struct {
	gyro       mgl32.Vec3
	accel      mgl32.Vec3
	timeDelta  time.Duration
	W, X, Y, Z float32
}{
	{mgl32.Vec3{-0.098402, 0.113307, 0.089605}, mgl32.Vec3{0.291015, 0.466064, 0.919677}, 101998848, 0.999987, 0.001520, 0.001696, 0.004570},
	{mgl32.Vec3{-0.072431, 0.107442, 0.188146}, mgl32.Vec3{0.308105, 0.470703, 0.866210}, 118718464, 0.999860, 0.004705, 0.003105, 0.015774},
	{mgl32.Vec3{-0.139940, 0.102521, 0.138474}, mgl32.Vec3{0.220703, 0.512939, 0.835449}, 118751232, 0.999684, 0.004576, 0.005488, 0.024090},
	{mgl32.Vec3{-0.040736, -0.009180, -0.071227}, mgl32.Vec3{0.313720, 0.447753, 0.873046}, 118751232, 0.999756, 0.009505, -0.000236, 0.019936},
	{mgl32.Vec3{-0.024365, 0.027157, 0.014364}, mgl32.Vec3{0.338867, 0.447998, 0.879882}, 119111680, 0.999659, 0.015212, -0.004066, 0.020854},
	{mgl32.Vec3{-0.018361, 0.364844, 0.292273}, mgl32.Vec3{0.340576, 0.499755, 0.885253}, 118882304, 0.998959, 0.021039, 0.012303, 0.038573},
	{mgl32.Vec3{-0.070825, -0.049253, -0.423173}, mgl32.Vec3{0.435791, 0.528076, 0.802734}, 118784000, 0.999624, 0.023462, 0.003927, 0.013658},
	{mgl32.Vec3{-0.272708, -0.211586, -0.557127}, mgl32.Vec3{0.295898, 0.428466, 0.964843}, 128745472, 0.999547, 0.013655, -0.014997, -0.022245},
	{mgl32.Vec3{0.044593, -0.217573, 0.241938}, mgl32.Vec3{0.344970, 0.361328, 0.902832}, 119144448, 0.999126, 0.022186, -0.034520, -0.007983},
	{mgl32.Vec3{-0.632891, -0.202650, -0.245009}, mgl32.Vec3{0.424316, 0.614013, 0.960693}, 118947840, 0.998392, -0.007435, -0.050704, -0.024233},
	{mgl32.Vec3{-0.075224, 0.564056, 0.519043}, mgl32.Vec3{0.365234, 0.343017, 1.502929}, 119209984, 0.999740, -0.005006, -0.021504, 0.005743},
	{mgl32.Vec3{-0.186942, 2.061286, 0.473106}, mgl32.Vec3{0.195800, 0.253906, 0.953369}, 118947840, 0.994810, -0.009583, 0.095914, 0.032588},
	{mgl32.Vec3{-0.344999, 1.959673, 0.744086}, mgl32.Vec3{0.240234, 0.330078, 0.849365}, 118849536, 0.975682, -0.023427, 0.203701, 0.077470},
	{mgl32.Vec3{-0.244608, 1.699881, -0.664185}, mgl32.Vec3{-0.015136, 0.266113, 0.801513}, 118882304, 0.954019, -0.047228, 0.293223, 0.040459},
	{mgl32.Vec3{0.538748, 0.166173, 0.292133}, mgl32.Vec3{-0.133056, -0.062744, 0.809082}, 128712704, 0.953485, -0.008674, 0.297549, 0.047493},
	{mgl32.Vec3{0.316638, 1.185358, 1.323710}, mgl32.Vec3{-0.381835, -0.161376, 0.218017}, 118947840, 0.921842, 0.023554, 0.369763, 0.113696},
	{mgl32.Vec3{-0.436751, 3.158225, -1.917873}, mgl32.Vec3{-0.779785, 0.723632, 0.461914}, 118816768, 0.843986, -0.055136, 0.532945, 0.024832},
	{mgl32.Vec3{0.754872, 2.368744, -3.165032}, mgl32.Vec3{-0.579101, 0.915771, 1.026123}, 118980608, 0.757156, -0.113314, 0.623251, -0.159475},
	{mgl32.Vec3{1.178446, 3.087121, -1.513061}, mgl32.Vec3{-0.490722, 0.572998, 0.962158}, 119013376, 0.624318, -0.081812, 0.722664, -0.285115},
	{mgl32.Vec3{2.317867, 4.363183, 0.595611}, mgl32.Vec3{-0.598144, 0.409179, 0.754882}, 119013376, 0.441172, 0.103338, 0.812025, -0.367837},
	{mgl32.Vec3{1.596016, 0.549273, 4.253856}, mgl32.Vec3{-0.451171, 1.020263, 0.454833}, 128548864, 0.484679, 0.371922, 0.728049, -0.310973},
	{mgl32.Vec3{1.813467, -0.215304, 4.363183}, mgl32.Vec3{-0.573486, 0.885253, -0.668701}, 118816768, 0.514618, 0.590584, 0.568547, -0.251265},
	{mgl32.Vec3{-0.101316, 3.396585, 1.697629}, mgl32.Vec3{-0.089599, 0.945556, -0.401367}, 118980608, 0.418329, 0.681812, 0.596298, -0.067541},
	{mgl32.Vec3{0.675111, -2.886984, 3.338798}, mgl32.Vec3{1.016357, 1.763427, 1.456787}, 119013376, 0.492029, 0.778955, 0.369920, -0.119571},
	{mgl32.Vec3{1.232098, -4.363323, 4.363183}, mgl32.Vec3{0.458496, 0.728759, -0.209472}, 118816768, 0.526964, 0.825129, 0.024972, -0.202106},
	{mgl32.Vec3{-0.281225, 0.703734, 3.864106}, mgl32.Vec3{0.403808, -0.416259, -1.379150}, 118915072, 0.564862, 0.812774, -0.136531, -0.041079},
	{mgl32.Vec3{-1.305733, -0.503597, 3.139463}, mgl32.Vec3{0.367187, -0.156250, -0.404541}, 119209984, 0.614429, 0.728850, -0.300182, 0.033838},
	{mgl32.Vec3{0.535956, -3.484864, 3.877162}, mgl32.Vec3{-0.025878, -0.104248, -1.035156}, 128811008, 0.478531, 0.651134, -0.588227, 0.031968},
	{mgl32.Vec3{-2.497898, -0.489739, 2.184314}, mgl32.Vec3{0.107666, -0.339111, -1.222900}, 119111680, 0.536263, 0.496799, -0.682277, -0.010514},
	{mgl32.Vec3{-1.078177, -0.482549, 2.557832}, mgl32.Vec3{-0.072265, 0.124511, -1.116210}, 119209984, 0.535253, 0.356478, -0.765691, 0.012018},
	{mgl32.Vec3{-1.152737, 2.826002, 1.089365}, mgl32.Vec3{0.085693, 0.442138, -0.970947}, 119078912, 0.669665, 0.267357, -0.691074, 0.049857},
	{mgl32.Vec3{-1.431571, 2.617208, 1.789364}, mgl32.Vec3{0.618408, 0.037841, -0.629394}, 118882304, 0.772972, 0.127801, -0.612907, 0.102598},
	{mgl32.Vec3{1.462865, 3.247988, 3.395782}, mgl32.Vec3{0.491943, 0.195556, -0.706298}, 118915072, 0.821496, 0.052163, -0.468165, 0.321318},
	{mgl32.Vec3{0.224362, 4.363183, 2.237320}, mgl32.Vec3{0.558105, 0.468750, -0.802490}, 128614400, 0.863703, -0.083997, -0.236604, 0.437012},
	{mgl32.Vec3{1.454470, 4.363183, -0.365507}, mgl32.Vec3{0.583740, -0.278564, -0.182373}, 118882304, 0.908700, -0.113367, 0.013925, 0.401520},
	{mgl32.Vec3{3.873828, 4.232162, 2.486449}, mgl32.Vec3{0.486816, 0.073242, 1.458984}, 119013376, 0.818650, 0.003907, 0.325747, 0.472955},
	{mgl32.Vec3{1.904276, 1.566066, -1.516255}, mgl32.Vec3{0.282714, 0.170166, 0.537353}, 118652928, 0.820306, 0.026671, 0.443968, 0.359554},
	{mgl32.Vec3{-0.367374, 3.625887, -1.994824}, mgl32.Vec3{-0.063476, 0.027099, 1.107666}, 118751232, 0.748324, -0.119049, 0.593844, 0.270533},
	{mgl32.Vec3{1.077636, 1.866734, -0.167778}, mgl32.Vec3{-0.178710, -0.001708, 1.126953}, 119013376, 0.689902, -0.105435, 0.684498, 0.210670},
	{mgl32.Vec3{2.396040, -0.356728, -0.005445}, mgl32.Vec3{-0.586425, 0.098388, 0.620361}, 119144448, 0.715739, -0.001268, 0.688961, 0.114230},
	{mgl32.Vec3{-0.513179, 2.390438, -2.527725}, mgl32.Vec3{-0.684814, -0.029541, 0.574707}, 128778240, 0.616188, -0.153032, 0.772361, 0.018744},
}

func TestMadgwick(t *testing.T) {
	filter := NewMadgwick(3.14159265358979 * (5.0 / 180.0)) // 5 deg/s
	const degToRad = 0.0174532925
	const maxDiff = 0.000001
	for i, row := range madgwickTestFixture {
		filter.Update(row.gyro, row.accel, row.timeDelta)
		if abs(filter.W-row.W) > maxDiff || abs(filter.X()-row.X) > maxDiff || abs(filter.Y()-row.Y) > maxDiff || abs(filter.Z()-row.Z) > maxDiff {
			t.Errorf("Row %d didn't match test fixture:\nfixture:  W=%f X=%f Y=%f Z=%f\ncomputed: W=%f X=%f Y=%f Z=%f",
				i, row.W, row.X, row.Y, row.Z, filter.W, filter.X(), filter.Y(), filter.Z())
		}
	}
}

func abs(x float32) float32 {
	if x < 0 {
		return -x
	}
	return x
}
