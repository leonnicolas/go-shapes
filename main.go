package main

import (
	"fmt"
	"math"
	"os"
	"sync"
	"time"
)

const (
	d0   = 15.0
	d1   = d0 * (float64(MAXX) / float64(MAXY))
	D    = 5.1
	MAXX = 80 // Screen width in ascii charackters
	MAXY = 40 // Screen hight in ascii charackters
)

type vector [3]float64

// light should be normalized
var light = vector{-1 / math.Sqrt(3), -1 / math.Sqrt(3), -1 / math.Sqrt(3)}

//var light = vector{0, 0, 1}

func (v vector) abs() (res float64) {
	for _, i := range v {
		res += i * i
	}
	math.Sqrt(res)
	return
}

func (v vector) normalize() (res vector) {
	abs := v.abs()
	for i := 0; i < 3; i++ {
		res[i] = v[i] / abs
	}
	return
}
func (v vector) multiply(m matrix) (res vector) {
	return m.multiplyWithVector(v)
}

func (v vector) scalarProduct(v2 vector) (res float64) {
	for i := 0; i < 3; i++ {
		res += v[i] * v2[i]
	}
	return
}

func (v vector) project() (x, y float64) {
	x = v[2] * d1 / (v[0] + D)
	y = v[1] * d0 / (v[0] + D)
	return
}

type matrix [3]vector

var identity matrix = matrix{
	vector{1, 0, 0},
	vector{0, 1, 0},
	vector{0, 0, 1},
}

func (m matrix) multiply(m2 matrix) (res matrix) {
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				res[i][j] += m[i][k] * m2[k][j]
			}
		}
	}
	return
}

func (m matrix) multiplyWithVector(v vector) (res vector) {
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			res[i] += m[i][j] * v[j]
		}
	}
	return
}

func (m matrix) rotX(a float64) (res matrix) {
	return matrix{
		vector{1, 0, 0},
		vector{0, math.Cos(a), -math.Sin(a)},
		vector{0, math.Sin(a), math.Cos(a)},
	}.multiply(m)
}

func (m matrix) rotY(a float64) (res matrix) {
	return matrix{
		vector{math.Cos(a), 0, math.Sin(a)},
		vector{0, 1, 0},
		vector{-math.Sin(a), 0, math.Cos(a)},
	}.multiply(m)
}

func (m matrix) rotZ(a float64) (res matrix) {
	return matrix{
		vector{math.Cos(a), -math.Sin(a), 0},
		vector{math.Sin(a), math.Cos(a), 0},
		vector{0, 0, 1},
	}.multiply(m)
}

type shape interface {
	next() vector       // returns the next point to draw
	normal() vector     // return the normal at the current point
	maxIterations() int // how many times after next returns the same points
}

type lightpoint struct {
	light    float64
	distance float64
}

func mapper(i float64) byte {
	i++
	i /= 2.0
	index := " ......,,,,,,----~:;=!****####$$$$@@@"
	return index[int(i*float64(len(index)-1))]
}

func run(s shape) {
	ibuf := make([]lightpoint, MAXX*MAXY)
	buf := make([]byte, MAXX)
	var wg sync.WaitGroup
	fmt.Print("\x1b[H")
	fmt.Print("\x1b[0J") // clear the screen
	for rotX, rotY, rotZ := 1.55, 0.0, 0.9; true; rotX, rotY, rotZ = rotX*1+math.Pi/192, rotY*1+math.Pi/185, rotZ*1+math.Pi/170 {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := 0; i < s.maxIterations(); i++ {
				rot := identity.rotX(rotX).rotY(rotY).rotZ(rotZ)
				v := s.next().multiply(rot)
				n := s.normal().multiply(rot)
				xp, yp := v.project()
				i, j := int(yp)+MAXY/2, int(xp)+MAXX/2
				if i*MAXX+j >= MAXY*MAXX || 0 >= i*MAXY+j {
					continue
				}
				e := &ibuf[i*MAXX+j]
				if t := n.normalize().scalarProduct(light); e.distance > v[0] {
					(*e).light = t
					(*e).distance = v[0]
				}
			}

			s := ""
			for k := 0; k < MAXY; k++ {
				for j := 0; j < MAXX; j++ {
					buf[j] = mapper(ibuf[k*MAXX+j].light)
				}
				s = fmt.Sprintf("%s\n%s", s, string(buf))

			}
			fmt.Print(s)
			for i := range ibuf {
				ibuf[i].light = 0.0
				ibuf[i].distance = 10000
			}
		}()
		time.Sleep(10000000)
		wg.Wait()
		fmt.Print("\x1b[H")
	}

	return
}

type donut struct {
	r1            float64
	r2            float64
	alpha, beta   float64
	alphaI, betaI int
}

func newDonut() *donut {
	return &donut{
		r1:     1.4,
		r2:     2.5,
		alphaI: 80,
		betaI:  180,
	}
}

func (d *donut) next() vector {
	if d.alpha >= 2*math.Pi {
		d.alpha = 0
		d.beta += 2 * math.Pi / float64(d.betaI)
	} else if d.beta >= 2*math.Pi {
		d.beta = 0
	} else {
		d.alpha += 2 * math.Pi / float64(d.alphaI)
	}
	return vector{d.r2 + d.r1*math.Cos(d.alpha), 0, d.r1 * math.Sin(d.alpha)}.multiply(identity.rotZ(d.beta))
}

func (d *donut) maxIterations() int {
	return d.alphaI*d.betaI + d.alphaI + d.betaI
}

func (d *donut) normal() vector {
	return vector{math.Cos(d.alpha), 0, math.Sin(d.alpha)}.multiply(identity.rotZ(d.beta))
}

type cuboid struct {
	a, b, c    float64
	aI, bI, cI int
	resolution int
}

func newCuboid(a, b, c float64) *cuboid {
	return &cuboid{
		a:          a,
		b:          b,
		c:          c,
		resolution: 40,
	}
}

func (c *cuboid) next() vector {
	if c.aI >= c.resolution {
		c.aI = 0
	} else if c.bI >= c.resolution {
		c.bI = 0
		c.aI++
	} else if c.cI >= c.resolution {
		c.cI = 0
		c.bI++
	} else if c.bI == 0 || c.bI == c.resolution-1 || c.aI == 0 || c.aI == c.resolution-1 {
		c.cI++
	} else {
		c.cI = c.resolution
	}
	return vector{c.a * float64(c.aI-c.resolution/2) / float64(c.resolution), c.b * float64(c.bI-c.resolution/2) / float64(c.resolution), c.c * float64(c.cI-c.resolution/2) / float64(c.resolution)}
}

func (c *cuboid) normal() (v vector) {
	switch {
	case c.aI == 0:
		v[0] = -1
	case c.bI == 0:
		v[1] = -1
	case c.cI == 0:
		v[2] = -1
	case c.aI >= c.resolution-1:
		v[0] = 1
	case c.bI >= c.resolution-1:
		v[1] = 1
	case c.cI >= c.resolution-1:
		v[2] = 1
	default:
		fmt.Println("weird")
		v[2] = 1
	}
	return
}

func (c *cuboid) maxIterations() int {
	return c.resolution * c.resolution * c.resolution
}

type tube struct {
	r, h, alpha, height      float64
	hresolution, rresolution int
}

func newTube(r, h float64, hr, rr int) *tube {
	return &tube{
		r:           r,
		h:           h,
		rresolution: rr,
		hresolution: hr,
	}
}

func (t *tube) next() vector {
	if t.alpha >= 2*math.Pi {
		t.alpha = 0
	} else if t.height >= t.h {
		t.height = 0
		t.alpha += 2 * math.Pi / float64(t.rresolution)
	} else {
		t.height += t.h / float64(t.hresolution)
	}
	return vector{t.r * math.Sin(t.alpha), t.r * math.Cos(t.alpha), t.height - t.h/2}
}

func (t *tube) normal() vector {
	return vector{math.Sin(t.alpha), math.Cos(t.alpha), 0}
}

func (t *tube) maxIterations() int {
	return t.hresolution * t.rresolution
}

func main() {
	var s shape

	switch os.Args[1] {
	case "cuboid":
		s = newCuboid(2.9, 2.9, 4.5)
	case "cube":
		s = newCuboid(3.9, 3.9, 3.9)
	case "donut":
		s = newDonut()
	case "tube":
		s = newTube(1, 7, 100, 40)
	case "ring":
		s = newTube(4, 0.5, 5, 200)
	default:
		os.Exit(1)
	}

	run(s)
}
