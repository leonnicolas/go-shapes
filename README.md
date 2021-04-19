# go-shapes

Draw cool and useless shapes to your terminal.

## Build and Run

Clone the repository and `cd` into it.

Run 
```bash 
go run main.go <donut|ring|tube|cube|cuboid>
```

## Make more shapes

By implementing the interface
```go
type shape interface {
	next() vector       // returns the next point to draw
	normal() vector     // return the normal at the current point
	maxIterations() int // how many times after next returns the same points
}
```
you can make more custom shapes.
You don't have to worry about the rotation, projection to the screen or mapping light values to ascii characters.
