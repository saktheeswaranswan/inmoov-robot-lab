angular.module('mrlapp.service.MailGui', []).controller('MailGuiCtrl', ['$scope', 'mrl', function($scope, mrl) {
    console.info('MailGuiCtrl')
    var _self = this
    var msg = this.msg

    // GOOD TEMPLATE TO FOLLOW
    this.updateState = function(service) {
        $scope.service = service
    }

    msg.subscribe(this)
}
])
