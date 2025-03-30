//go:build linux

package ioctl

import "C"
import (
	"errors"
	"syscall"
)

// It is inspired from
// https://github.com/junka/ioctl/blob/main/ioctl.go
//
// See also:
// https://www.circlemud.org/jelson/software/fusd/docs/node31.html

const (
	iocNone = iota
	iocWrite
	iocRead
)

const (
	iocNrBits   = 8
	iocTypeBits = 8
	iocSizeBits = 14

	iocNrShift   = 0
	iocSizeShift = iocTypeShift + iocTypeBits
	iocTypeShift = iocNrShift + iocNrBits
	iocDirShift  = iocSizeShift + iocSizeBits
)

func ioc(dir, tp, nr, size uint64) uint64 {
	return (dir << iocDirShift) | (tp << iocTypeShift) | (nr << iocNrShift) | (size << iocSizeShift)
}

func IO(tp, nr uint64) uint64 {
	return ioc(iocNone, tp, nr, 0)
}

func IOW(tp, nr, size uint64) uint64 {
	return ioc(iocWrite, tp, nr, size)
}

func IOR(tp, nr, size uint64) uint64 {
	return ioc(iocRead, tp, nr, size)
}

func IOWR(tp, nr, size uint64) uint64 {
	return ioc(iocRead|iocWrite, tp, nr, size)
}

func Ioctl(fd int, request uint64, arg ...uintptr) error {
	var err error
	if len(arg) == 0 {
		_, _, err = syscall.Syscall(syscall.SYS_IOCTL, uintptr(fd), uintptr(request), 0)
	} else if len(arg) == 1 {
		_, _, err = syscall.Syscall(syscall.SYS_IOCTL, uintptr(fd), uintptr(request), arg[0])
	} else if len(arg) == 4 {
		_, _, err = syscall.Syscall6(syscall.SYS_IOCTL, uintptr(fd), uintptr(request), arg[0], arg[1], arg[2], arg[3])
	}
	if errors.Is(err, syscall.Errno(0)) {
		return nil
	}
	return err
}

func Open(path string, mode int, perm uint32) (int, error) {
	return syscall.Open(path, mode|syscall.O_NONBLOCK, perm)
}

func Close(fd int) error {
	return syscall.Close(fd)
}
