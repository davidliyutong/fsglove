package tests

import (
	"github.com/spf13/cobra"
	"hand_apiserver/internal/cmd"
	"testing"
	"time"
)

func TestServe(t *testing.T) {
	var serve = &cobra.Command{Use: "root", RunE: cmd.ServeCmdRunE}
	cmd.ServeCmdFlags(serve)
	go serve.Execute()
	time.Sleep(60 * time.Second)
	return
}
